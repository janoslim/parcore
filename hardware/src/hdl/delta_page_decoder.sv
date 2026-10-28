`timescale 1ns / 1ps

`include "libstf_macros.svh"

import lynxTypes::AXI_DATA_BITS;
import libstf::*;

/**
 * DeltaPageDecoder decodes the DELTA_BINARY_PACKED values of one Data Page V1 per configuration.
 * Added in the gpu-smartssd-olap fork (2026-10-29): upstream ParCore has no DELTA decoder.
 *
 * Written from the Parquet specification (Encodings.md, "Delta Encoding (DELTA_BINARY_PACKED = 5)"):
 *   page     = <definition levels> <header> <block>*
 *              As everywhere in ParCore, the column is OPTIONAL without nulls. The levels are a uint32 length
 *              and that many bytes, and are skipped here.
 *   header   = <block size in values> <miniblocks in a block> <total value count> <first value>
 *              These are ULEB128 varints. The first value is zigzag encoded.
 *   block    = <min delta> <one bit-width byte per miniblock> <miniblocks>
 *              min delta is a zigzag ULEB128. Miniblock m holds (block size / miniblocks) deltas, bit-packed
 *              little endian at its bit width. The last miniblock with values is padded to full length. The
 *              miniblocks after it have no bytes, and their bit-width bytes can hold anything.
 *   value[0] = first value, value[i] = value[i - 1] + min delta + delta[i], wrapping at the type's width.
 * Supported:
 *   - INT32 and INT64 columns, with bit widths up to 32 and 64.
 *   - 1 to 64 miniblocks per block, with values per miniblock a multiple of 16 (the specification requires 32).
 * The total value count comes from the DELTA header. Padding values and the bytes after the page's last value
 * are discarded.
 *
 * Datapath (ParCore's 64-byte beat):
 *   Every step unpacks 8 INT64 or 16 INT32 deltas, one output beat of values. A bit-packed group of 8 deltas is
 *   exactly W bytes at bit width W, so every step starts on a byte and consumes W (INT64) or 2W (INT32) bytes,
 *   at most 64.
 *   front end  A window of up to three input beats and a byte offset into the first. A state machine skips the
 *              levels and parses the page and block headers; every varint takes two cycles (capture, use). It
 *              then issues one record per step.
 *   back end   A pipeline without stalls: unpack the lanes (E1), add min delta (E2), prefix-sum the lanes
 *              (P1..P4), add the running value (E7).
 *   output     A FIFO. The front end issues a record only while the FIFO has a free entry for it (credits), so
 *              the back end never waits for out.ready.
 * Throughput: a block header costs 3 cycles (min delta 2, bit widths 1) and every step 1 cycle.
 * Every value is decoded; filtering stays behind this decoder.
 */
module DeltaPageDecoder #(
    parameter NUM_BYTES = AXI_DATA_BITS / 8
) (
    input logic clk,
    input logic rst_n,

    ready_valid_i.s conf, // #(type_t) one per page: the column's physical type, INT32_T or INT64_T
    ndata_i.s       in,   // #(data8_t, NUM_BYTES) the page: dense beats from byte 0, last on its final beat
    typed_ndata_i.m out   // #(NUM_BYTES) the page's values, contiguous from byte 0 per beat, last on its final beat
);

`ASSERT_ELAB(NUM_BYTES == 64)

`RESET_RESYNC // Reset pipelining

// ── Geometry ────────────────────────────────────────────────────────────────────────────────────────────
localparam int BEAT_BITS       = 8 * NUM_BYTES;
localparam int INT64_LANES     = NUM_BYTES / 8;              // INT64 deltas per step
localparam int INT32_LANES     = NUM_BYTES / 4;              // INT32 deltas per step
localparam int NARROW_LANES    = INT32_LANES - INT64_LANES;  // lanes 8..15, which only INT32 steps use
localparam int MAX_MINIBLOCKS  = NUM_BYTES;                  // a block's bit widths are read from one view
localparam int VARINT_BYTES    = 10;                         // the longest ULEB128 of a 64-bit value
localparam int FIFO_DEPTH      = 32;                         // output entries, more than the records in flight
localparam int FIFO_INDEX_BITS = $clog2(FIFO_DEPTH);
localparam int PIPELINE_STAGES = 8;                          // R0, E1, E2, P1, P2, P3, P4, E7
localparam int PREFIX_LEVELS   = 4;                          // INT32 needs four levels, INT64 three

typedef logic [BEAT_BITS - 1:0] beat_t;

typedef enum logic [1:0] {
    RECORD_FIRST, // the page's first value, in lane 0
    RECORD_STEP,  // one step of bit-packed deltas
    RECORD_END    // no values: closes the output of a page that has none (or cannot be decoded)
} record_kind_t;

typedef enum logic [3:0] {
    ST_IDLE,
    ST_LEVELS_LENGTH,  // the uint32 length of the definition levels
    ST_SKIP_LEVELS,    // skip the levels, up to 64 bytes per cycle
    ST_VARINT_CAPTURE, // a header varint, cycle 1: capture its bytes and its length
    ST_VARINT_USE,     // cycle 2: consume it and act on it (varint_purpose)
    ST_WIDTHS,         // the block's bit-width bytes
    ST_STEPS,          // bit-packed deltas, one step per cycle
    ST_ERROR,          // an unsupported or malformed page: close its output with an empty last beat
    ST_DRAIN           // discard the rest of the page up to its last beat
} state_t;

typedef enum logic [2:0] {
    VARINT_BLOCK_SIZE,
    VARINT_MINIBLOCKS,
    VARINT_TOTAL,
    VARINT_FIRST_VALUE,
    VARINT_MIN_DELTA
} varint_purpose_t;

// What a record carries through the back end besides its bytes.
typedef struct packed {
    logic         valid;
    record_kind_t kind;
    logic [4:0]   lanes; // values the record outputs: 1 for a first value, 0 for an end record
    logic         int64;
    logic         last;  // the record's beat is the page's last one
    type_t        typ;
} control_t;

typedef struct packed {
    logic                   last;
    type_t                  typ;
    logic [NUM_BYTES - 1:0] keep;
    beat_t                  data;
} entry_t;

// ── Helpers ─────────────────────────────────────────────────────────────────────────────────────────────

// Length (1..VARINT_BYTES) of the ULEB128 varint at byte 0 of `bytes`, 0 if none of its bytes ends it.
function automatic logic [3:0] varint_length(input logic [8 * VARINT_BYTES - 1:0] bytes);
    logic [3:0] length;
    length = 4'd0;
    // From the last byte down, so the first byte without the continuation bit wins.
    for (int i = VARINT_BYTES - 1; i >= 0; i--) begin
        if (!bytes[8 * i + 7]) length = 4'(i + 1);
    end
    return length;
endfunction

// Low 64 bits of the ULEB128 varint of `length` bytes at byte 0 of `bytes`.
function automatic logic [63:0] varint_value(input logic [8 * VARINT_BYTES - 1:0] bytes, input logic [3:0] length);
    logic [7 * VARINT_BYTES - 1:0] value;
    value = '0;
    for (int i = 0; i < VARINT_BYTES; i++) begin
        if (i < length) value[7 * i +: 7] = bytes[8 * i +: 7];
    end
    return value[63:0];
endfunction

function automatic logic [63:0] zigzag_decode(input logic [63:0] value);
    return {1'b0, value[63:1]} ^ {64{value[0]}};
endfunction

// Whether a miniblock bit width fits the column's type.
function automatic logic width_supported(input logic [7:0] width, input logic int64);
    return int64 ? width <= 8'd64 : width <= 8'd32;
endfunction

// Bytes of one step: 8 deltas of `width` bits (INT64) or 16 (INT32).
function automatic logic [6:0] bytes_per_step(input logic [7:0] width, input logic int64);
    return int64 ? width[6:0] : {width[5:0], 1'b0};
endfunction

// Byte keep of `count` values.
function automatic logic [NUM_BYTES - 1:0] lane_keep(input logic [4:0] count, input logic int64);
    logic [NUM_BYTES - 1:0] keep;
    for (int b = 0; b < NUM_BYTES; b++) begin
        keep[b] = int64 ? (b / 8 < count) : (b / 4 < count);
    end
    return keep;
endfunction

// ── Front end: the input window ─────────────────────────────────────────────────────────────────────────
// Up to three beats. window0 holds the next unconsumed byte at `offset`; the 64 bytes from it (`view`) lie in
// window0 and window1. A third beat lets a step consume a whole beat per cycle while the next one arrives.
beat_t      window0, window1, window2;
logic [1:0] beats;       // valid beats in the window
(* max_fanout = 32 *) logic [5:0] offset;
logic       last_loaded; // the page's last beat has been accepted

beat_t view;
assign view = BEAT_BITS'({window1, window0} >> (8 * offset));

// The 64 bytes from offset are in the window, or the page has no more bytes to come. A well-formed page
// holds every byte a step or header reads, padding included, so beyond its end nothing used is read.
logic window_ready;
assign window_ready = beats >= 2'd2 || last_loaded;

// ── Front end: page and block state ─────────────────────────────────────────────────────────────────────
state_t          state;
varint_purpose_t varint_purpose;
type_t           value_type;       // the configured type, passed on as out.typ
logic            is_int64;         // 8 INT64 lanes per step, else 16 INT32 lanes

logic [31:0] skip_left;            // definition-level bytes still to skip
logic [31:0] block_size;           // deltas per block
logic [6:0]  miniblocks;           // miniblocks per block (1..64)
logic [10:0] step_weight;          // miniblocks x lanes: the steps of a miniblock add up to block_size
logic [31:0] values_total;         // the header's total value count
logic [31:0] values_left;          // deltas of the page not yet issued
logic [63:0] min_delta;            // of the current block
logic [8 * MAX_MINIBLOCKS - 1:0] widths; // bit widths of the block's miniblocks, the current one in byte 0
logic [6:0]  miniblocks_left;      // miniblocks of the block, the current one included
logic [31:0] miniblock_weight;     // steps of the current miniblock so far x step_weight
logic [6:0]  width;                // bit width of the current miniblock
logic [6:0]  step_bytes;           // bytes per step: width (INT64) or 2 x width (INT32)
logic [8 * VARINT_BYTES - 1:0] varint_bytes; // captured by ST_VARINT_CAPTURE
logic [3:0]  varint_bytes_length;

logic [FIFO_INDEX_BITS:0] credits; // free FIFO entries no issued record has claimed
logic has_credit;
assign has_credit = credits != '0;

logic [4:0]  lanes;                // deltas per step
logic        step_is_last;         // this step holds the page's last deltas
logic [31:0] next_miniblock_weight;
logic        miniblock_ends;       // this step is the last one of its miniblock
logic [7:0]  next_width;           // bit width of the block's next miniblock
logic [63:0] varint_decoded;

assign lanes                 = is_int64 ? 5'(INT64_LANES) : 5'(INT32_LANES);
assign step_is_last          = values_left <= 32'(lanes);
assign next_miniblock_weight = miniblock_weight + 32'(step_weight);
assign miniblock_ends        = next_miniblock_weight >= block_size;
assign next_width            = widths[15:8];
assign varint_decoded        = varint_value(varint_bytes, varint_bytes_length);

// ── Front end: what this cycle consumes and issues ──────────────────────────────────────────────────────
logic [6:0]   consume;     // bytes consumed from offset (0..64)
logic         issue;       // a record enters the back end
record_kind_t issue_kind;
logic [4:0]   issue_lanes;
logic         issue_last;

always_comb begin
    consume     = '0;
    issue       = 1'b0;
    issue_kind  = RECORD_STEP;
    issue_lanes = '0;
    issue_last  = 1'b0;

    case (state)
        ST_LEVELS_LENGTH: begin
            if (window_ready) consume = 7'd4;
        end
        ST_SKIP_LEVELS: begin
            if (window_ready) consume = skip_left > 32'd64 ? 7'd64 : 7'(skip_left);
        end
        ST_VARINT_USE: begin
            // The first value is issued as its varint is consumed, so that varint waits for a credit.
            if (varint_purpose != VARINT_FIRST_VALUE || has_credit) consume = 7'(varint_bytes_length);
            if (varint_purpose == VARINT_FIRST_VALUE && has_credit && varint_bytes_length != 4'd0) begin
                issue       = 1'b1;
                issue_kind  = values_total == 32'd0 ? RECORD_END : RECORD_FIRST;
                issue_lanes = values_total == 32'd0 ? 5'd0 : 5'd1;
                issue_last  = values_total <= 32'd1;
            end
        end
        ST_WIDTHS: begin
            if (window_ready) consume = miniblocks;
        end
        ST_STEPS: begin
            if (window_ready && has_credit) begin
                consume     = step_bytes;
                issue       = 1'b1;
                issue_lanes = step_is_last ? 5'(values_left) : lanes;
                issue_last  = step_is_last;
            end
        end
        ST_ERROR: begin
            if (has_credit) begin
                issue      = 1'b1;
                issue_kind = RECORD_END;
                issue_last = 1'b1;
            end
        end
        default: ;
    endcase
end

// ── Front end: window updates ───────────────────────────────────────────────────────────────────────────
logic       accept;      // an input beat enters the window
logic       retire;      // window0 is consumed up to its end
logic [6:0] offset_sum;  // offset + consume, 0..127: at most one beat retires per cycle
logic [1:0] write_slot;  // where an accepted beat goes: behind the beats that stay

// Ready depends on registers only. No beat of the next page enters before this page is drained.
assign in.ready   = state != ST_IDLE && !last_loaded && (state == ST_DRAIN || beats != 2'd3);
assign conf.ready = state == ST_IDLE;

assign accept     = in.valid && in.ready;
assign offset_sum = 7'(offset) + consume;
assign retire     = offset_sum[6] && beats != 2'd0;
assign write_slot = beats - 2'(retire);

always_ff @(posedge clk) begin
    if (retire) begin
        window0 <= window1;
        window1 <= window2;
    end
    if (accept) begin
        case (write_slot)
            2'd0:    window0 <= in.data;
            2'd1:    window1 <= in.data;
            default: window2 <= in.data;
        endcase
    end
end

// ── Front end: state machine ────────────────────────────────────────────────────────────────────────────
always_ff @(posedge clk) begin
    if (reset_synced == 1'b0) begin
        state       <= ST_IDLE;
        beats       <= '0;
        offset      <= '0;
        last_loaded <= 1'b0;
    end else begin
        // Step 1: the window. Draining discards it; otherwise the consumed bytes leave it and an accepted beat
        // joins it.
        if (state == ST_DRAIN) begin
            beats  <= '0;
            offset <= '0;
        end else begin
            beats  <= beats - 2'(retire) + 2'(accept);
            offset <= offset_sum[5:0];
        end
        if (accept && in.last) last_loaded <= 1'b1;

        // Step 2: the page
        case (state)
            ST_IDLE: begin
                if (conf.valid) begin
                    value_type <= conf.data;
                    is_int64   <= GET_TYPE_WIDTH(conf.data) == 64;
                    state      <= ST_LEVELS_LENGTH;
                end
            end
            ST_LEVELS_LENGTH: begin
                if (window_ready) begin
                    skip_left <= view[31:0];
                    state     <= ST_SKIP_LEVELS;
                end
            end
            ST_SKIP_LEVELS: begin
                if (window_ready) begin
                    skip_left <= skip_left - 32'(consume);
                    if (skip_left <= 32'd64) begin
                        varint_purpose <= VARINT_BLOCK_SIZE;
                        state          <= ST_VARINT_CAPTURE;
                    end
                end
            end
            ST_VARINT_CAPTURE: begin
                if (window_ready) begin
                    varint_bytes        <= view[8 * VARINT_BYTES - 1:0];
                    varint_bytes_length <= varint_length(view[8 * VARINT_BYTES - 1:0]);
                    state               <= ST_VARINT_USE;
                end
            end
            ST_VARINT_USE: begin
                if (varint_bytes_length == 4'd0) begin
                    state <= ST_ERROR; // none of the 10 bytes ends the varint
                end else begin
                    case (varint_purpose)
                        VARINT_BLOCK_SIZE: begin
                            block_size     <= varint_decoded[31:0];
                            varint_purpose <= VARINT_MINIBLOCKS;
                            state          <= ST_VARINT_CAPTURE;
                        end
                        VARINT_MINIBLOCKS: begin
                            miniblocks     <= varint_decoded[6:0];
                            step_weight    <= is_int64 ? {varint_decoded[7:0], 3'd0} : {varint_decoded[6:0], 4'd0};
                            varint_purpose <= VARINT_TOTAL;
                            // 1..64 miniblocks of a non-empty block, or the page is not supported
                            if (varint_decoded == 64'd0 || varint_decoded > 64'd64 || block_size == 32'd0) begin
                                state <= ST_ERROR;
                            end else begin
                                state <= ST_VARINT_CAPTURE;
                            end
                        end
                        VARINT_TOTAL: begin
                            values_total   <= varint_decoded[31:0];
                            varint_purpose <= VARINT_FIRST_VALUE;
                            state          <= ST_VARINT_CAPTURE;
                        end
                        VARINT_FIRST_VALUE: begin
                            if (has_credit) begin
                                values_left    <= values_total - 32'd1;
                                varint_purpose <= VARINT_MIN_DELTA;
                                state          <= values_total <= 32'd1 ? ST_DRAIN : ST_VARINT_CAPTURE;
                            end
                        end
                        default: begin // VARINT_MIN_DELTA, the start of a block
                            min_delta <= zigzag_decode(varint_decoded);
                            state     <= ST_WIDTHS;
                        end
                    endcase
                end
            end
            ST_WIDTHS: begin
                if (window_ready) begin
                    widths           <= view[8 * MAX_MINIBLOCKS - 1:0];
                    width            <= view[6:0];
                    step_bytes       <= bytes_per_step(view[7:0], is_int64);
                    miniblocks_left  <= miniblocks;
                    miniblock_weight <= '0;
                    state            <= width_supported(view[7:0], is_int64) ? ST_STEPS : ST_ERROR;
                end
            end
            ST_STEPS: begin
                if (window_ready && has_credit) begin
                    values_left <= values_left - 32'(lanes);
                    if (step_is_last) begin
                        state <= ST_DRAIN;
                    end else if (miniblock_ends) begin
                        miniblock_weight <= '0;
                        if (miniblocks_left == 7'd1) begin
                            varint_purpose <= VARINT_MIN_DELTA;
                            state          <= ST_VARINT_CAPTURE;
                        end else begin
                            miniblocks_left <= miniblocks_left - 7'd1;
                            widths          <= widths >> 8;
                            width           <= next_width[6:0];
                            step_bytes      <= bytes_per_step(next_width, is_int64);
                            // Only miniblocks with values are checked: the others' widths can be anything.
                            if (!width_supported(next_width, is_int64)) state <= ST_ERROR;
                        end
                    end else begin
                        miniblock_weight <= next_miniblock_weight;
                    end
                end
            end
            ST_ERROR: begin
                if (has_credit) state <= ST_DRAIN; // the end record is issued in always_comb
            end
            ST_DRAIN: begin
                if (last_loaded || (accept && in.last)) begin
                    last_loaded <= 1'b0;
                    state       <= ST_IDLE;
                end
            end
            default: state <= ST_IDLE;
        endcase
    end
end

// ── Back end: the control of every stage, R0 (issued record) to E7 (output entry) ────────────────────
control_t control [PIPELINE_STAGES];

always_ff @(posedge clk) begin
    if (reset_synced == 1'b0) begin
        for (int stage = 0; stage < PIPELINE_STAGES; stage++) control[stage].valid <= 1'b0;
    end else begin
        control[0] <= '{valid: issue, kind: issue_kind, lanes: issue_lanes, int64: is_int64, last: issue_last,
                        typ: value_type};
        for (int stage = 1; stage < PIPELINE_STAGES; stage++) control[stage] <= control[stage - 1];
    end
end

// ── R0: the record's bytes. A step's bytes start at its first byte; a first value is in the low 64 bits ──
beat_t       r0_bytes;
(* max_fanout = 32 *) logic [6:0] r0_width;
logic [63:0] r0_min_delta;

always_ff @(posedge clk) begin
    r0_bytes[BEAT_BITS - 1:64] <= view[BEAT_BITS - 1:64];
    r0_bytes[63:0]             <= issue_kind == RECORD_STEP ? view[63:0] : zigzag_decode(varint_decoded);
    r0_width                   <= width;
    r0_min_delta               <= min_delta;
end

// ── E1: unpack. Lane k of a step is bits [k*W, k*W + W) of its bytes ───────────────────────────────────
// Every candidate width is pre-masked, so an output bit only chooses among the widths that reach it.
logic [63:0] wide_candidates   [INT64_LANES][65];  // lanes 0..7 at bit widths 0..64
logic [31:0] narrow_candidates [NARROW_LANES][33]; // lanes 8..15 at bit widths 0..32
logic [6:0]  narrow_width;                         // INT64 steps leave the INT32-only lanes at width 0
logic [63:0] e1_wide_next [INT64_LANES], e1_wide [INT64_LANES];
logic [31:0] e1_narrow_next [NARROW_LANES], e1_narrow [NARROW_LANES];
logic [63:0] e1_min_delta;

assign narrow_width = r0_width > 7'd32 ? 7'd0 : r0_width;

for (genvar k = 0; k < INT64_LANES; k++) begin : gen_wide_lanes
    for (genvar w = 0; w <= 64; w++) begin : gen_widths
        localparam logic [64:0] LOW_BITS = (65'd1 << w) - 65'd1;
        assign wide_candidates[k][w] = r0_bytes[k * w +: 64] & LOW_BITS[63:0];
    end
    // A first value travels in lane 0 as it is.
    if (k == 0) begin : gen_first_value
        assign e1_wide_next[k] = control[0].kind == RECORD_STEP ? wide_candidates[k][r0_width] : r0_bytes[63:0];
    end else begin : gen_delta
        assign e1_wide_next[k] = wide_candidates[k][r0_width];
    end
end
for (genvar j = 0; j < NARROW_LANES; j++) begin : gen_narrow_lanes
    for (genvar w = 0; w <= 32; w++) begin : gen_widths
        localparam logic [32:0] LOW_BITS = (33'd1 << w) - 33'd1;
        assign narrow_candidates[j][w] = r0_bytes[(j + INT64_LANES) * w +: 32] & LOW_BITS[31:0];
    end
    assign e1_narrow_next[j] = narrow_candidates[j][narrow_width];
end

always_ff @(posedge clk) begin
    e1_wide      <= e1_wide_next;
    e1_narrow    <= e1_narrow_next;
    e1_min_delta <= r0_min_delta;
end

// ── E2: add the block's min delta. The low 32 bits of a 64-bit sum are the INT32 sum ───────────────────
logic [63:0] e2_wide_next [INT64_LANES], e2_wide [INT64_LANES];
logic [31:0] e2_narrow_next [NARROW_LANES], e2_narrow [NARROW_LANES];

for (genvar k = 0; k < INT64_LANES; k++) begin : gen_add_wide
    if (k == 0) begin : gen_first_value
        assign e2_wide_next[k] = control[1].kind == RECORD_STEP ? e1_wide[k] + e1_min_delta : e1_wide[k];
    end else begin : gen_delta
        assign e2_wide_next[k] = e1_wide[k] + e1_min_delta;
    end
end
for (genvar j = 0; j < NARROW_LANES; j++) begin : gen_add_narrow
    assign e2_narrow_next[j] = e1_narrow[j] + e1_min_delta[31:0];
end

always_ff @(posedge clk) begin
    e2_wide   <= e2_wide_next;
    e2_narrow <= e2_narrow_next;
end

// ── P1..P4: inclusive prefix sums over the lanes (Kogge-Stone) ─────────────────────────────────────────
// Level l adds the lane 2^l below. INT64 needs levels 1, 2, 4 and passes through the fourth, so both types
// leave P4 together.
logic [31:0] e2_lanes32 [INT32_LANES];
logic [63:0] prefix64_next [PREFIX_LEVELS][INT64_LANES], prefix64 [PREFIX_LEVELS][INT64_LANES];
logic [31:0] prefix32_next [PREFIX_LEVELS][INT32_LANES], prefix32 [PREFIX_LEVELS][INT32_LANES];

for (genvar k = 0; k < INT32_LANES; k++) begin : gen_int32_lanes
    if (k < INT64_LANES) begin : gen_low
        assign e2_lanes32[k] = e2_wide[k][31:0];
    end else begin : gen_high
        assign e2_lanes32[k] = e2_narrow[k - INT64_LANES];
    end
end

for (genvar l = 0; l < PREFIX_LEVELS; l++) begin : gen_prefix_levels
    localparam int DISTANCE64 = l < 3 ? 1 << l : 0; // 0: pass through
    localparam int DISTANCE32 = 1 << l;
    for (genvar k = 0; k < INT64_LANES; k++) begin : gen_int64
        logic [63:0] own, below;
        if (l == 0) begin : gen_from_e2
            assign own = e2_wide[k];
            if (k >= DISTANCE64) begin : gen_add
                assign below = e2_wide[k - DISTANCE64];
            end else begin : gen_none
                assign below = '0;
            end
        end else begin : gen_from_level
            assign own = prefix64[l - 1][k];
            if (DISTANCE64 != 0 && k >= DISTANCE64) begin : gen_add
                assign below = prefix64[l - 1][k - DISTANCE64];
            end else begin : gen_none
                assign below = '0;
            end
        end
        assign prefix64_next[l][k] = own + below;
    end
    for (genvar k = 0; k < INT32_LANES; k++) begin : gen_int32
        logic [31:0] own, below;
        if (l == 0) begin : gen_from_e2
            assign own = e2_lanes32[k];
            if (k >= DISTANCE32) begin : gen_add
                assign below = e2_lanes32[k - DISTANCE32];
            end else begin : gen_none
                assign below = '0;
            end
        end else begin : gen_from_level
            assign own = prefix32[l - 1][k];
            if (k >= DISTANCE32) begin : gen_add
                assign below = prefix32[l - 1][k - DISTANCE32];
            end else begin : gen_none
                assign below = '0;
            end
        end
        assign prefix32_next[l][k] = own + below;
    end
end

always_ff @(posedge clk) begin
    prefix64 <= prefix64_next;
    prefix32 <= prefix32_next;
end

// ── E7: add the running value; the output entry ─────────────────────────────────────────────────────────
control_t    p4;                   // the record leaving P4
logic [63:0] accumulator;          // the page's last value so far
logic [63:0] value64 [INT64_LANES];
logic [31:0] value32 [INT32_LANES];
beat_t       values_int64, values_int32;
entry_t      e7_entry;

assign p4 = control[PIPELINE_STAGES - 2];

for (genvar k = 0; k < INT64_LANES; k++) begin : gen_values64
    assign value64[k] = p4.kind == RECORD_STEP ? accumulator + prefix64[PREFIX_LEVELS - 1][k]
                                               : prefix64[PREFIX_LEVELS - 1][k];
    assign values_int64[64 * k +: 64] = value64[k];
end
for (genvar k = 0; k < INT32_LANES; k++) begin : gen_values32
    assign value32[k] = p4.kind == RECORD_STEP ? accumulator[31:0] + prefix32[PREFIX_LEVELS - 1][k]
                                               : prefix32[PREFIX_LEVELS - 1][k];
    assign values_int32[32 * k +: 32] = value32[k];
end

always_ff @(posedge clk) begin
    // The accumulator is the page's value before the next step: the first value, then a step's last lane.
    // The page's final step may be partial; the accumulator is not used after it.
    if (p4.valid && p4.kind == RECORD_FIRST) begin
        accumulator <= p4.int64 ? value64[0] : {32'd0, value32[0]};
    end else if (p4.valid && p4.kind == RECORD_STEP) begin
        accumulator <= p4.int64 ? value64[INT64_LANES - 1] : {32'd0, value32[INT32_LANES - 1]};
    end
    e7_entry <= '{last: p4.last, typ: p4.typ, keep: lane_keep(p4.lanes, p4.int64),
                  data: p4.int64 ? values_int64 : values_int32};
end

// ── Output FIFO. Credits: an issued record claims an entry, an entry leaving the memory frees one ─────
(* ram_style = "distributed" *) entry_t fifo_memory [FIFO_DEPTH];
logic [FIFO_INDEX_BITS:0] write_pointer, read_pointer; // one bit wider than an index: equal means empty
entry_t head;                                          // the oldest entry, shown on out
logic   head_valid;
logic   memory_empty, head_free, memory_read;

assign memory_empty = write_pointer == read_pointer;
assign head_free    = !head_valid || out.ready;
assign memory_read  = head_free && !memory_empty;

always_ff @(posedge clk) begin
    if (control[PIPELINE_STAGES - 1].valid) fifo_memory[write_pointer[FIFO_INDEX_BITS - 1:0]] <= e7_entry;
    if (memory_read) head <= fifo_memory[read_pointer[FIFO_INDEX_BITS - 1:0]];
end

always_ff @(posedge clk) begin
    if (reset_synced == 1'b0) begin
        write_pointer <= '0;
        read_pointer  <= '0;
        head_valid    <= 1'b0;
        credits       <= (FIFO_INDEX_BITS + 1)'(FIFO_DEPTH);
    end else begin
        write_pointer <= write_pointer + (FIFO_INDEX_BITS + 1)'(control[PIPELINE_STAGES - 1].valid);
        read_pointer  <= read_pointer + (FIFO_INDEX_BITS + 1)'(memory_read);
        if (head_free) head_valid <= !memory_empty;
        credits <= credits - (FIFO_INDEX_BITS + 1)'(issue) + (FIFO_INDEX_BITS + 1)'(memory_read);
    end
end

assign out.data  = head.data;
assign out.keep  = head.keep;
assign out.typ   = head.typ;
assign out.last  = head.last;
assign out.valid = head_valid;

endmodule

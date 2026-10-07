`timescale 1ns / 1ps

`include "libstf_macros.svh"
`include "lynx_macros.svh"

import lynxTypes::*;
import libstf::*;
import parcore::*;

module RunDecoder #(
    parameter type data_t,
    parameter NUM_ELEMENTS,
    parameter NUM_BYTES = AXI_DATA_BITS / 8
) (
    input logic clk,
    input logic rst_n,

    ready_valid_i.s conf, // #(run_decoder_config_t)
    ndata_i.s in,         // #(data8_t, NUM_BYTES)

    ndata_i.m out         // #(data_t, NUM_ELEMENTS)
);

`RESET_RESYNC // Reset pipelining

// This is a simplification made to have better timing closure. Under this
// assumption, all bitpacking encodings are bit_width * NUM_ELEMENTS bits
// long. Because NUM_ELEMENTS is divisible by 8, then so is the number of bits
// of each BPE run. Thanks to that, there is never any bit-level offset
// between BPE sections, and we can thus avoid keeping track of that,
// simplifying indexing and avoiding some divisions (which would in turn
// require DSPs on the critical path).
`ASSERT_ELAB(NUM_ELEMENTS % 8 == 0);

localparam int MAX_IN_TRANSIT = 8;
localparam int DATA_SIZE = ($bits(data_t) + 7) / 8;
localparam int VARINT_OFFSET_COMPUTATION_WIDTH = 18;

// ------- Input extraction ------
data8_t[NUM_BYTES - 1:0] in_data;
assign in_data = in.data;
logic[NUM_BYTES - 1:0] in_keep;
assign in_keep = in.keep;

run_decoder_config_t conf_data;
assign conf_data = conf.data;

// ------- State declaration -----

typedef enum logic [2:0] {
    ST_IDLE,
    ST_HEADER,
    ST_HEADER2,
    ST_DECODE_RLE,
    ST_DECODE_BPE
} state_t;
state_t state;

logic store_in_second_half;
data8_t[NUM_BYTES * 2 - 1:0] data;
logic[NUM_BYTES * 2 - 1:0] keep;
logic last_received;
bit_width_t bit_width;
logic [BPE_MASK_SIZE - 1:0] bit_width_bpe_mask;
offset_t offset;
data32_t remaining_values;

// ------- Combinatorial state ---
data8_t[NUM_BYTES * 2 - 1:0] next_data;
logic[NUM_BYTES * 2 - 1:0] next_keep;
logic next_last_received;

// n bits for bit_width_t, + log2(NUM_ELEMENTS) bits
// as this value is the result of bit_width * NUM_ELEMENTS;
logic[$clog2(NUM_ELEMENTS) + $bits(bit_width_t) - 1:0] packed_databeat_bits;
logic[$clog2(NUM_ELEMENTS) + $bits(bit_width_t) - 1 - $clog2(8):0] packed_databeat_bytes;
logic[$clog2(8) + $bits(bit_width_t) - 1 - $clog2(8):0] eight_packed_databeat_bytes;
offset_t varint_offset;

// ------- Registered lookahead (RunDecoderTiming, decoder-timing-fix findings.md section 11) -----
// Post-route, the worst RunDecoder paths decided a handshake from a keep lookup at a computed index
// (keep[offset + packed_databeat_bytes - 1], keep[offset + i]) and then, in the same cycle, read the
// next varint at an index that itself depended on the varint just decoded: 10-14 logic levels
// (diag-decoder-d0hse-timing-03/endpoints-post_route_phys_opt.tsv, ooc proxy decoder-hse_r1-01).
// Each register below holds a value the original computed combinationally. It is written at every
// point where that value's inputs change, so the ports behave as before cycle for cycle; only the
// position of the logic relative to the flops changes.
//
// bvb_r == keep[offset] && keep[offset + packed_databeat_bytes - 1]       (read in ST_DECODE_BPE)
// rvb_r == AND over i < DATA_SIZE of (i >= rle_width || keep[offset + i]) (read in ST_DECODE_RLE)
// Both are read only in the decode states, and every entry into a decode state goes through
// goto_decode(), which writes them.
logic bvb_r, rvb_r;
// vbase_r    == varint_offset + 1 + rle_width     (mod 128)
// vbase_m[k] == varint_offset + 1 + k * bit_width (mod 128), k = 0..3
// A run that starts right after the varint at varint_offset is followed by the next varint at
// varint_offset + length + payload, with payload rle_width (RLE) or k * bit_width (a bit-packed run of
// k <= 3 groups, the only one goto_decode_bpe() reads ahead for). As flops, these bases let the
// next-varint window be read from registers while the decoded length selects inside it afterwards.
// They follow varint_offset every cycle except the one cycle after goto_decode_bpe() enters a run with
// two or more 16-value inputs left, when no window read can happen.
offset_t vbase_r;
offset_t vbase_m[4];
offset_t bw_x3; // 3 * bit_width mod 128, captured with the configuration

// ------- Output declaration -----
typedef enum logic {
  OUTPUT_RLE,
  OUTPUT_BPE
} output_t;
valid_i #(output_t) next_out(clk, reset_synced), curr_out(clk, reset_synced);
// This signal is used to apply backpressure from the output queue. In case
// the queue gets full, this signal will be low and the next decoding will be
// paused until there's space in the queue to store it.
logic can_decode_next;

always_comb begin
    // Default assignments prevent latch inference on next_out.data.
    next_out.data  = OUTPUT_RLE;
    next_out.valid = 1'b0;

    case (state)
        ST_DECODE_RLE: begin
            next_out.data = OUTPUT_RLE;
            next_out.valid = rle_in.valid && rle_in.ready;
        end

        ST_DECODE_BPE: begin
            next_out.data = OUTPUT_BPE;
            next_out.valid = bpe_in.valid && bpe_in.ready;
        end
    endcase
end

// ------- State declaration (decoders) -----
logic [3:0] rle_width;
rle_count_t rle_count;

// Assuming worse case: 0...2**$bits(data_t)-1 repeated twice.
// In that case, each elements takes ceil($bits(data_t)/8) bytes to be
// encoded. Subtract by log2(NUM_ELEMNETS) for number of BPE inputs.
localparam int ENC_BYTES   = ($bits(data_t) + 7) / 8;          // ceil($bits/8)
localparam int BASE_BITS   = ($bits(data_t) - 1) + ENC_BYTES;  // worst-case size

typedef logic [BASE_BITS - $clog2(NUM_ELEMENTS) - 1:0] bpe_remaining_inputs_t;

// The value in bpe_count is computed by << 3 the value in the header
// (ignoring the LSB). This means that it'll be a multiple of 8.
// For runs where the number of values encoded in BPE is not a multiple of 8,
// the bpe_count is lower-bounded by the total number of values in the page,
// which is found in conf_data.num_values.
bpe_count_t bpe_count;
bpe_remaining_inputs_t bpe_remaining_inputs; 


// ------- Combinatorial state (decoders) ---
// This is a bit-level view of the data
logic [NUM_BYTES * 8 * 2 - 1:0] bpe_data;
generate
for (genvar i = 0; i < NUM_BYTES * 2; i++) begin
    assign bpe_data[(i+1) * 8 - 1:i * 8] = data[i];
end
endgenerate

// ------- Header varint decoding

// To have some valid varint input, we must either have:
// - up to 4 valid bytes
// - at least 1 valid byte if we've received last. We assume the input is
// correct.
valid_i #(data8_t[VARINT_NUM_BYTES - 1:0]) varint_in(clk, reset_synced);
valid_i #(varint_t) varint_out(clk, reset_synced);

VarintDecoder inst_varint_decoder (
    .in(varint_in),
    .out(varint_out)
);

data8_t[3:0] expected_varint_data;
assign expected_varint_data = data[varint_offset +: 4];

assert property (@(posedge clk) disable iff (!rst_n) !varint_in.valid || (expected_varint_data ==? varint_in.data))
else $fatal(1, "Varint input data does not match the data at the current offset. In state %d, at varint_offset 0x%x, offset 0x%x, expected 0b%b (%x), got 0b%b (%x)", state, varint_offset, offset, expected_varint_data, expected_varint_data, varint_in.data, varint_in.data);

// Combinatorial shift values from the varint value used to compute rle_count
// and bpe_count.
logic varint_encoding;
assign varint_encoding = varint_out.data.value[0];
typedef enum logic {
    ENCODING_RLE = 0,
    ENCODING_BPE = 1
} varint_encoding_t;
rle_count_t varint_no_encoding;
assign varint_no_encoding = varint_out.data.value >> 1;
bpe_count_t varint_no_encoding_bytes;
assign varint_no_encoding_bytes = varint_no_encoding  << 3;

// ------- RLE decoding
tagged_i #(data_t, $bits(rle_count_t)) rle_in(clk, reset_synced);
ndata_i #(data_t, NUM_ELEMENTS) rle_out(clk, reset_synced);

ExpandRLE #(
    .data_t(data_t),
    .NUM_ELEMENTS(NUM_ELEMENTS)
) inst_expand_rle (
    .clk(clk),
    .rst_n(reset_synced),

    .in(rle_in),
    .out(rle_out)
);

assign rle_in.tag = rle_count;
generate
for (genvar i = 0; i < DATA_SIZE; i++) begin
    // We need to copy bit-by-bit here as for value sizes that are not
    // multiple of eight, DATA_SIZE will be an over approximation of how many
    // bytes are required. For example, for $bits(data_t) = 18, DATA_SIZE = 3,
    // but we can't access indexes 23:18, only 17:16 for the last byte.
    for (genvar b = 0; b < 8 && i * 8 + b < $bits(data_t); b++) begin
        assign rle_in.data[i * 8 + b] = (i < rle_width) ? data[offset+i][b] : '0;
    end
end
endgenerate
// rvb_r is the registered &rle_in_valid_bits of the original (see its declaration).
assign rle_in.valid = can_decode_next &&state == ST_DECODE_RLE && rvb_r;
logic rle_needs_more_input;
// The original |rle_needs_to_buffer_bits is exactly ~&rle_in_valid_bits.
assign rle_needs_more_input = ~rvb_r && ~last_received;

// ------- BPE decoding
bpe_config_t bpe_in_tag;
assign bpe_in_tag.bit_width = bit_width;
assign bpe_in_tag.mask = bit_width_bpe_mask;
assign bpe_in_tag.count = bpe_count;

tagged_i #(logic [$bits(data_t) * NUM_ELEMENTS - 1:0], $bits(bpe_config_t)) bpe_in(clk, reset_synced);
ndata_i  #(data_t, NUM_ELEMENTS) bpe_out(clk, reset_synced);

ExpandBPE #(
    .data_t(data_t),
    .NUM_ELEMENTS(NUM_ELEMENTS),
    .MAX_IN_TRANSIT(MAX_IN_TRANSIT)
) inst_expand_bpe (
    .clk(clk),
    .rst_n(reset_synced),

    .in(bpe_in),
    .out(bpe_out)
);

// NOTE: ExpandBPE doesn't look at the keep signals.
assign bpe_in.data = bpe_data[offset * 8 +: $bits(data_t) * NUM_ELEMENTS];
assign bpe_in.last = bpe_count <= NUM_ELEMENTS;
assign bpe_in.tag = bpe_in_tag;
// OPTIMIZATION: here we're only checking for the first and last bit of the
// desired keep region, to avoid a wide & over several bits. bvb_r holds that
// pair for the current offset (see its declaration).
logic bpe_valid_bytes;
assign bpe_valid_bytes = bvb_r;

assign bpe_in.valid = can_decode_next && state == ST_DECODE_BPE && (bpe_valid_bytes || last_received) && bpe_count > 0;

// We want to take more input if some of the keep bytes are not high, and
// only if we haven't already consumed the last databeat.
logic bpe_needs_more_input;
assign bpe_needs_more_input = ~bpe_valid_bytes && ~last_received;

// ------- Lookahead candidates for bvb_r / rvb_r -----
// keep reads past the double buffer return 0: that is what the original reads one cycle later from the
// half update_offset() has just zeroed (keep_zext). Indices use 8-bit arithmetic, exact for bit widths
// up to 63 (offset <= 127, packed_databeat_bytes <= 126; the RunDecoder's ids are 19 bits). Where the
// original index ran past the buffer itself, or below 0, it read X, which the interfaces' not-undefined
// assertions exclude from passing runs. data_twice/keep_twice repeat the buffer so a `+:` window wraps
// mod 128 inside the mux select, with no adder in front of it.
data8_t[NUM_BYTES * 4 - 1:0] data_twice;
logic[NUM_BYTES * 4 - 1:0] keep_twice, keep_zext;
assign data_twice = {data, data};
assign keep_twice = {keep, keep};
assign keep_zext = {{(NUM_BYTES * 2){1'b0}}, keep};

// (a) store_input(): the half named by store_in_second_half takes in.keep and offset stays. A store
// never coincides with an offset update: the decode states and ST_HEADER2 accept input only in cycles
// in which they cannot advance.
logic[NUM_BYTES * 2 - 1:0] store_keep;
assign store_keep = store_in_second_half ? {in_keep, keep[NUM_BYTES - 1:0]}
                                         : {keep[NUM_BYTES * 2 - 1:NUM_BYTES], in_keep};
logic[NUM_BYTES * 4 - 1:0] store_keep_zext;
assign store_keep_zext = {{(NUM_BYTES * 2){1'b0}}, store_keep};
logic[7:0] store_end; // offset + packed_databeat_bytes - 1
assign store_end = 8'(offset) + 8'(packed_databeat_bytes) - 8'd1;
logic[DATA_SIZE - 1:0] store_keep_at;
assign store_keep_at = store_keep_zext[offset +: DATA_SIZE];
logic bvb_store, rvb_store;
logic[DATA_SIZE - 1:0] rvb_store_bits;
assign bvb_store = store_keep[offset] && store_keep_zext[store_end];
generate
for (genvar i = 0; i < DATA_SIZE; i++) begin : gen_store_lookahead
    assign rvb_store_bits[i] = (i >= rle_width) || store_keep_at[i];
end
endgenerate
assign rvb_store = &rvb_store_bits;

// (b) advance_bpe(): update_offset(offset + packed_databeat_bytes). The next cycle's
// keep[trim(n)] and keep[trim(n) + packed_databeat_bytes - 1], after the optional shift, are this
// buffer's bytes n and n + packed_databeat_bytes - 1 with zeros past it.
offset_t advance_offset;
assign advance_offset = offset + packed_databeat_bytes;
logic[7:0] advance_end; // advance_offset + packed_databeat_bytes - 1
assign advance_end = 8'(advance_offset) + 8'(packed_databeat_bytes) - 8'd1;
logic bvb_advance;
assign bvb_advance = keep[advance_offset] && keep_zext[advance_end];

// (c) goto_decode(): update_offset(varint_offset + varint length), one candidate per length 1..4;
// the decoded length selects afterwards (oav_l[L] is offset_after_varint for length L). The keep bits
// are read as windows at registered bases (no adder in front of the mux): offset_after_varint wraps
// mod 128 (keep_twice), the run bytes after it read zero past the buffer (keep_zext).
logic[6:0] goto_keep_wrap, goto_keep_zero;
assign goto_keep_wrap = keep_twice[varint_offset +: 7];
assign goto_keep_zero = keep_zext[varint_offset +: 7];
// keep at offset_after_varint + packed_databeat_bytes - 1 = goto_end + length - 1, and the same mod 128
// when offset_after_varint itself wrapped (exact for bit widths up to 63; the RunDecoder's ids are 19 bits).
logic[7:0] goto_end;
assign goto_end = 8'(varint_offset) + 8'(packed_databeat_bytes);
logic[3:0] goto_keep_end, goto_keep_end_wrap;
assign goto_keep_end = keep_zext[goto_end +: 4];
assign goto_keep_end_wrap = keep_zext[goto_end[$bits(offset_t) - 1:0] +: 4];
logic[4:1] bvb_goto_l, rvb_goto_l;
offset_t oav_l[4:1];
generate
for (genvar L = 1; L <= 4; L++) begin : gen_goto_lookahead
    logic wrap;
    logic[DATA_SIZE - 1:0] rb;
    assign oav_l[L] = varint_offset + offset_t'(L);
    assign wrap = 32'(varint_offset) + L >= NUM_BYTES * 2;
    for (genvar i = 0; i < DATA_SIZE; i++) begin : gen_rle_bits
        assign rb[i] = (i >= rle_width) || (wrap ? goto_keep_wrap[L + i] : goto_keep_zero[L + i]);
    end
    assign rvb_goto_l[L] = &rb;
    assign bvb_goto_l[L] = goto_keep_wrap[L] && (wrap ? goto_keep_end_wrap[L - 1] : goto_keep_end[L - 1]);
end
endgenerate

// ------- Next-varint windows -----
// Every write of varint_in reads 4 bytes, plus the keep bits of the validity test, at one of a few
// indices. The original computed the index in place and called update_varint_data() /
// next_varint_valid() with it; here each window is read in parallel from flops and the state machine
// only selects. Windows are `+:` part-selects of the buffer written twice in a row (data_twice,
// keep_twice above), so an index wraps mod 128 inside the mux select with no adder in front of it.
// Where the original read past the double buffer it read X; the two agree wherever it is defined.

// ST_IDLE / ST_HEADER: in.data zero-extended to the double buffer, at the configured offset (ST_IDLE)
// or at varint_offset (ST_HEADER, which holds that same configured offset).
offset_t win_in_base;
assign win_in_base = state == ST_IDLE ? conf_data.offset : varint_offset;
data8_t[NUM_BYTES * 4 - 1:0] in_data_zext;
logic[NUM_BYTES * 4 - 1:0] in_keep_zext;
assign in_data_zext = {{(NUM_BYTES * 3){8'h00}}, in_data};
assign in_keep_zext = {{(NUM_BYTES * 3){1'b0}}, in_keep};
data8_t[3:0] win_in;
logic[3:0] win_in_keep;
logic win_in_valid;
assign win_in = in_data_zext[win_in_base +: 4];
assign win_in_keep = in_keep_zext[win_in_base +: 4];
assign win_in_valid = win_in_keep[0] && (in.last || &win_in_keep[3:1]);

// ST_HEADER2 reads next_data at varint_offset; advance_bpe() with one 16-value input left and
// finish_bpe() without a decoded varint read the buffer at varint_offset or varint_offset + 64.
data8_t[NUM_BYTES * 2 - 1:0] in_data_twice;
logic[NUM_BYTES * 2 - 1:0] in_keep_twice;
assign in_data_twice = {in_data, in_data};
assign in_keep_twice = {in_keep, in_keep};
data8_t[3:0] win_at0, win_at1, win_next, win_in_at;
logic[3:0] win_at0_keep, win_at1_keep, win_next_keep, win_in_at_keep;
logic win_at0_valid, win_at1_valid, win_next_valid;
assign win_at0 = data_twice[varint_offset +: 4];
assign win_at0_keep = keep_twice[varint_offset +: 4];
assign win_at1 = data_twice[(varint_offset ^ offset_t'(NUM_BYTES)) +: 4]; // varint_offset + 64 mod 128
assign win_at1_keep = keep_twice[(varint_offset ^ offset_t'(NUM_BYTES)) +: 4];
assign win_in_at = in_data_twice[varint_offset[$clog2(NUM_BYTES) - 1:0] +: 4];
assign win_in_at_keep = in_keep_twice[varint_offset[$clog2(NUM_BYTES) - 1:0] +: 4];
generate
for (genvar j = 0; j < 4; j++) begin : gen_win_next
    // next_data takes in.data in the half store_in_second_half names; byte varint_offset + j lies in
    // the upper half when bit 6 of that sum is set.
    offset_t x;
    assign x = varint_offset + offset_t'(j);
    assign win_next[j] = x[$clog2(NUM_BYTES)] == store_in_second_half ? win_in_at[j] : win_at0[j];
    assign win_next_keep[j] = x[$clog2(NUM_BYTES)] == store_in_second_half ? win_in_at_keep[j] : win_at0_keep[j];
end
endgenerate
assign win_at0_valid = win_at0_keep[0] && (last_received || &win_at0_keep[3:1]);
assign win_at1_valid = win_at1_keep[0] && (last_received || &win_at1_keep[3:1]);
assign win_next_valid = win_next_keep[0] && (next_last_received || &win_next_keep[3:1]);

// goto_decode(): the next varint starts at varint_offset + length + payload, i.e. at
// goto_base + length - 1 with goto_base one of the registered vbase_*. The 7-byte window at goto_base
// covers all four lengths; the length then picks 4 bytes out of it.
logic goto_bpe;
logic[1:0] goto_groups;
assign goto_bpe = varint_out.data.value[0];    // == varint_encoding
assign goto_groups = varint_out.data.value[2:1]; // 8-value groups when the run has <= 3 of them
offset_t goto_base;
assign goto_base = goto_bpe ? vbase_m[goto_groups] : vbase_r;
data8_t[6:0] goto_window;
logic[6:0] goto_window_keep;
assign goto_window = data_twice[goto_base +: 7];
assign goto_window_keep = keep_twice[goto_base +: 7];

// New bases once goto_decode() moves varint_offset to the next varint. The new varint_offset is the
// next varint's index (goto_base + length - 1) with bit 6 cleared by trim_offset() for a bit-packed
// run, or offset_after_varint's bit 6 removed for an RLE run (trim_offset(oav) + rle_width); adding the
// base constants commutes with that bit flip mod 128.
offset_t goto_sum_r, goto_sum_m[4];
assign goto_sum_r = goto_base + offset_t'(rle_width);
assign goto_sum_m[0] = goto_base;
assign goto_sum_m[1] = goto_base + offset_t'(bit_width);
assign goto_sum_m[2] = goto_base + offset_t'(2 * bit_width);
assign goto_sum_m[3] = goto_base + bw_x3;

data8_t[3:0] win_goto;
logic win_goto_valid;
logic bvb_goto, rvb_goto;
offset_t goto_oav;
always_comb begin
    case (varint_out.data.length)
        VARINT_LENGTH_BITS'(1): begin
            win_goto = goto_window[3:0];
            win_goto_valid = goto_window_keep[0] && (last_received || &goto_window_keep[3:1]);
            bvb_goto = bvb_goto_l[1];
            rvb_goto = rvb_goto_l[1];
            goto_oav = oav_l[1];
        end
        VARINT_LENGTH_BITS'(2): begin
            win_goto = goto_window[4:1];
            win_goto_valid = goto_window_keep[1] && (last_received || &goto_window_keep[4:2]);
            bvb_goto = bvb_goto_l[2];
            rvb_goto = rvb_goto_l[2];
            goto_oav = oav_l[2];
        end
        VARINT_LENGTH_BITS'(3): begin
            win_goto = goto_window[5:2];
            win_goto_valid = goto_window_keep[2] && (last_received || &goto_window_keep[5:3]);
            bvb_goto = bvb_goto_l[3];
            rvb_goto = rvb_goto_l[3];
            goto_oav = oav_l[3];
        end
        // A decoded varint (the only kind goto_decode() consumes) has length 1..4.
        default: begin
            win_goto = goto_window[6:3];
            win_goto_valid = goto_window_keep[3] && (last_received || &goto_window_keep[6:4]);
            bvb_goto = bvb_goto_l[4];
            rvb_goto = rvb_goto_l[4];
            goto_oav = oav_l[4];
        end
    endcase
end

offset_t goto_index, goto_flip;
assign goto_index = goto_base + offset_t'(varint_out.data.length) - 7'd1;
assign goto_flip = {goto_bpe ? goto_index[$bits(offset_t) - 1] : goto_oav[$bits(offset_t) - 1],
                    {($bits(offset_t) - 1){1'b0}}};
offset_t goto_new_r, goto_new_m[4];
assign goto_new_r = (goto_sum_r + offset_t'(varint_out.data.length)) ^ goto_flip;
generate
for (genvar k = 0; k < 4; k++) begin : gen_goto_new
    assign goto_new_m[k] = (goto_sum_m[k] + offset_t'(varint_out.data.length)) ^ goto_flip;
end
endgenerate

// The bases implied by the current varint_offset; the default next value of vbase_*.
offset_t refresh_r, refresh_m[4];
assign refresh_r = varint_offset + offset_t'(rle_width) + 7'd1;
assign refresh_m[0] = varint_offset + 7'd1;
assign refresh_m[1] = varint_offset + offset_t'(bit_width) + 7'd1;
assign refresh_m[2] = varint_offset + offset_t'(2 * bit_width) + 7'd1;
assign refresh_m[3] = varint_offset + bw_x3 + 7'd1;


// ------- Combinatorial input ---
always_comb begin
    next_data = data;
    next_keep = keep;

    if (in.valid) begin
        if (store_in_second_half) begin
            next_data[NUM_BYTES * 2 - 1:NUM_BYTES] = in_data;
            next_keep[NUM_BYTES * 2 - 1:NUM_BYTES] = in_keep;
        end else begin
            next_data[NUM_BYTES - 1:0] = in_data;
            next_keep[NUM_BYTES - 1:0] = in_keep;
        end
    end
    next_last_received = in.last;
end

// ------- State machine ---------
function offset_t trim_offset(offset_t offset);
    // trim_offset = offset >= NUM_BYTES ? offset - NUM_BYTES : offset;
    trim_offset = offset[$bits(offset_t) - 2:0];
endfunction

task store_input();
    data <= next_data;
    keep <= next_keep;
    last_received <= next_last_received;

    store_in_second_half <= ~store_in_second_half;

    // Lookahead at the unchanged offset over the keep just stored.
    bvb_r <= bvb_store;
    rvb_r <= rvb_store;
endtask

task update_offset(input offset_t next_offset);
    `ifndef SYNTHESIS
    if (next_offset < offset) begin
        $fatal(1, "Attempted to decrement offset in update_offset, going from %d to %d", offset, next_offset);
    end
    `endif

    // If the new offset is beyond the midpoint of the data buffer, which
    // holds two databeats, then we rewrite the offset and move the second
    // half of the buffer into the first, zeroing the second.
    if (next_offset >= NUM_BYTES) begin
        data[NUM_BYTES - 1:0] <= data[NUM_BYTES * 2 - 1:NUM_BYTES];
        keep[NUM_BYTES - 1:0] <= keep[NUM_BYTES * 2 - 1:NUM_BYTES];

        data[NUM_BYTES * 2 - 1:NUM_BYTES] <=  '{default: 'x};
        keep[NUM_BYTES * 2 - 1:NUM_BYTES] <=  '0;

        store_in_second_half <= ~store_in_second_half;

        if (varint_offset >= NUM_BYTES) begin
            varint_offset <= trim_offset(varint_offset);
            // varint_offset - NUM_BYTES flips bit 6 of every mod-128 base derived from it.
            vbase_r <= refresh_r ^ offset_t'(NUM_BYTES);
            for (int k = 0; k < 4; k++) begin
                vbase_m[k] <= refresh_m[k] ^ offset_t'(NUM_BYTES);
            end
        end
    end
    offset <= trim_offset(next_offset);
endtask

task reset();
    state <= ST_IDLE;
    data <= 'x;
    keep <= '0;
    last_received <= '0;
    store_in_second_half <= 0;

    bit_width <= 'x;
    bit_width_bpe_mask <= 'x;
    packed_databeat_bits <= 'x;
    packed_databeat_bytes <= 'x;
    eight_packed_databeat_bytes <= 'x;
    varint_offset <= 'x;
    varint_in.valid <= 0;
    offset <= 'x;
    remaining_values <= 'x;

    rle_width <= 'x;
    rle_count <= 'x;
    bpe_count <= 'x;

    bvb_r <= 1'b0;
    rvb_r <= 1'b0;
endtask

task goto_decode(input data32_t remaining_values);
    logic less_remaining_values_than_next_bpe_count;
    bpe_count_t next_bpe_count, next_bpe_padded_count;

    less_remaining_values_than_next_bpe_count = remaining_values < varint_no_encoding_bytes;
    next_bpe_count = less_remaining_values_than_next_bpe_count ? remaining_values : varint_no_encoding_bytes;
    next_bpe_padded_count = varint_no_encoding_bytes;

    `ifndef SYNTHESIS
    if (~varint_out.valid) begin
        $fatal(1, "Called goto_decode() when varint_out.valid = %b", varint_out.valid);
    end
    `endif

    // goto_oav == offset_after_varint: varint_offset + length chosen among precomputed sums.
    update_offset(goto_oav);
    // Lookahead at the run's first input.
    bvb_r <= bvb_goto;
    rvb_r <= rvb_goto;
    if (varint_encoding == ENCODING_BPE) begin
        state <= ST_DECODE_BPE;

        // Compute BPE properties
        bpe_count <= next_bpe_count;

        goto_decode_bpe(next_bpe_padded_count, goto_oav);
    end else begin
        // Compute RLE properties
        rle_count <= varint_no_encoding;

        goto_decode_rle(goto_oav);
    end
endtask

task goto_decode_bpe(
    input bpe_count_t bpe_padded_cnt,
    input offset_t offst
);
    bpe_remaining_inputs_t next_bpe_remaining_inputs; 
    logic [$clog2(NUM_ELEMENTS) - 1:0] values_in_extra_input;
    offset_t next_varint_offset_increment_extra, next_varint_offset_increment, next_varint_offset;

    next_bpe_remaining_inputs = bpe_padded_cnt / NUM_ELEMENTS;
    values_in_extra_input = bpe_padded_cnt % NUM_ELEMENTS;
    next_varint_offset_increment_extra = values_in_extra_input > 0 ? eight_packed_databeat_bytes : 0;
    next_varint_offset_increment = (next_bpe_remaining_inputs * packed_databeat_bytes) + next_varint_offset_increment_extra;
    next_varint_offset = offst + next_varint_offset_increment;

    `ifndef SYNTHESIS
    if (next_varint_offset_increment != offset_t'((bpe_padded_cnt * bit_width) / 8)) begin
        $fatal(1, "goto_decode_bpe() computed the wrong next_varint_offset_increment, expected %d, got %d", offset_t'((bpe_padded_cnt * bit_width) / 8), next_varint_offset_increment);
    end
    `endif

    `ifndef SYNTHESIS
    if (next_varint_offset != offset_t'(offst + ((bpe_padded_cnt * bit_width) / 8))) begin
        $fatal(1, "goto_decode_bpe() computed the wrong next_varint_offset, expected %d, got %d", offset_t'(offst + ((bpe_padded_cnt * bit_width) / 8)), next_varint_offset);
    end
    `endif

    // BPE could contain so many values that the offset would go beyond two
    // databeats, in that case, we set the varint position but we don't make
    // it valid.
    varint_offset <= trim_offset(next_varint_offset);
    bpe_remaining_inputs <= next_bpe_remaining_inputs;

    if (next_bpe_remaining_inputs <= 1) begin
        // Here we use next_varint_offset (which may be > NUM_BYTES) as if
        // that's the case, in this databeat we also moved the offset forward
        // and shifted the data, so we store the varint_offset trimmed
        // (outside of this loop) but compute the correct varint_in data to
        // match.
        // With at most one 16-value input, next_varint_offset is
        // offset_after_varint + groups * bit_width = goto_base + length - 1,
        // the start of win_goto.
        varint_in.data <= win_goto;
        varint_in.valid <= win_goto_valid;
        vbase_r <= goto_new_r;
        for (int k = 0; k < 4; k++) begin
            vbase_m[k] <= goto_new_m[k];
        end
    end else begin
        // vbase_* take the default refresh from this cycle's varint_offset and
        // are refreshed from the new one in the next cycle; this run has at
        // least two inputs left, so no goto_decode() reads them in between.
        varint_in.valid <= 0;
    end

    state <= ST_DECODE_BPE;
endtask

task advance_bpe();
    bpe_count_t next_bpe_count;
    bpe_remaining_inputs_t  next_bpe_remaining_inputs;
    offset_t next_offset;

    next_bpe_count = bpe_count - NUM_ELEMENTS;
    next_bpe_remaining_inputs = bpe_remaining_inputs - 1;
    next_offset = offset + packed_databeat_bytes;

    bpe_count <= next_bpe_count;
    remaining_values <= remaining_values - NUM_ELEMENTS;
    bpe_remaining_inputs <= next_bpe_remaining_inputs;
    update_offset(next_offset);
    bvb_r <= bvb_advance;

    `ifndef SYNTHESIS
    if (bpe_remaining_inputs == 0 || bpe_in.last) begin
        $fatal(1, "advance_decode() has been called on the final BPE input");
    end
    `endif

    if (bpe_remaining_inputs == 1) begin
        logic [$bits(offset_t):0] next_next_offset;
        logic increment_varint_offset;
        offset_t actual_varint_offset;
        logic next_varint_in_valid;

        next_next_offset = next_offset + packed_databeat_bytes;
        // The varint offset for the next next cycle, when the next and final
        // bpe encoded chunks will have been decoded, depends on whether we're
        // moving the next_offset beyond NUM_BYTES (and thus shifting the
        // value in `data` by 512 bits) or not. If that's the case we want to
        // use a +64byte offest as in the current cycle the shift has not
        // happened yet.
        //
        // The `next_offset > varint_offset` condition ensures that we're only
        // looking into the second input half if the next offset is beyond the
        // varint_offset, meaning that it is in the second half of the stream.
        // The varint offset shall never be > 64.
        increment_varint_offset = next_offset > varint_offset && next_next_offset >= NUM_BYTES;
        actual_varint_offset = increment_varint_offset ? varint_offset + NUM_BYTES : varint_offset;
        // The two candidate windows (at varint_offset and varint_offset + 64)
        // are read in parallel; the comparison above only selects.
        next_varint_in_valid = increment_varint_offset ? win_at1_valid : win_at0_valid;

        varint_in.data <= increment_varint_offset ? win_at1 : win_at0;
        varint_in.valid <= next_varint_in_valid;

        // We only want to store the offset if we haven't trimmed the input in
        // this cycle and if we haven't set a valid varint input already.
        // Note that if we have just trimmed the output, the varint_offset is
        // already correct.
        if (~next_varint_in_valid && next_offset < NUM_BYTES) begin
            varint_offset <= actual_varint_offset;
            vbase_r <= increment_varint_offset ? refresh_r ^ offset_t'(NUM_BYTES) : refresh_r;
            for (int k = 0; k < 4; k++) begin
                vbase_m[k] <= increment_varint_offset ? refresh_m[k] ^ offset_t'(NUM_BYTES) : refresh_m[k];
            end
        end
    end
endtask

task finish_bpe();
    data32_t next_remaining_values;
    next_remaining_values = remaining_values - bpe_count;

    remaining_values <= next_remaining_values;
    // next_remaining_values == 0 exactly when the operands are equal (bpe_count zero-extends), which
    // keeps the 32-bit subtract's carry chain off the decision.
    if (remaining_values == data32_t'(bpe_count)) begin
        reset();
    end else if (varint_out.valid) begin
        // If the varint for the next databeat is already valid and parsed, we can
        // move forward to the next decode, otherwise we store the varint offset
        // which we're trying to decode and move to a state waiting for more input).
        goto_decode(next_remaining_values);
    end else begin
        offset_t actual_varint_offset;

        // varint_offset is stored trimmed (mod NUM_BYTES). If it isn't ahead of
        // offset the run wrapped into the second buffer half, so add NUM_BYTES:
        // update_offset() then performs the shift and lands on the trimmed
        // varint_offset, and the varint data is read from the pre-shift
        // second-half location. (Mirrors the rem==1 path in advance_bpe().)
        actual_varint_offset = (varint_offset <= offset) ? varint_offset + NUM_BYTES
                                                         : varint_offset;

        // NOTE: this update here is needed as this last BPE decoding might
        // have involved receiving more input, meaning that the varint may now
        // be valid.
        varint_in.data <= (varint_offset <= offset) ? win_at1 : win_at0;
        varint_in.valid <= (varint_offset <= offset) ? win_at1_valid : win_at0_valid;
        update_offset(actual_varint_offset);

        // If ~varint_out.valid we need to fetch more input to
        // satisfy it.
        state <= ST_HEADER2;
    end
endtask

task goto_decode_rle(input offset_t offst);
    state <= ST_DECODE_RLE;
    varint_offset <= trim_offset(offst) + rle_width;

    // NOTE: these are using the current offset, not the trimmed value.
    // This is because, if there has been a change in the offset in this cycle,
    // the data will be shifted but only from the next cycle, so when indexing
    // data and keep to populate the varint decoder, we need to use the
    // current offset.
    // offst + rle_width == goto_base + length - 1, the start of win_goto.
    varint_in.data <= win_goto;
    varint_in.valid <= win_goto_valid;
    vbase_r <= goto_new_r;
    for (int k = 0; k < 4; k++) begin
        vbase_m[k] <= goto_new_m[k];
    end
endtask

task finish_rle();
    data32_t next_remaining_values;
    next_remaining_values = remaining_values - rle_count;

    remaining_values <= next_remaining_values;
    // See finish_bpe(): equality instead of the subtract's zero test.
    if (remaining_values == data32_t'(rle_count)) begin
        reset();
    end else if (varint_out.valid) begin
        // If the varint for the next databeat is already valid and parsed, we can
        // move forward to the next decode, otherwise we store the varint offset
        // which we're trying to decode and move to a state waiting for more input).
        goto_decode(next_remaining_values);
    end else begin
        // If ~varint_out.valid we need to fetch more input to
        // satisfy it.
        state <= ST_HEADER2;
    end
endtask

always_ff @(posedge clk) begin
    if (reset_synced == 1'b0) begin
        reset();
    end else begin
        // Default: the window bases follow varint_offset (overridden below
        // wherever varint_offset itself is written).
        vbase_r <= refresh_r;
        for (int k = 0; k < 4; k++) begin
            vbase_m[k] <= refresh_m[k];
        end

        if (in.ready && in.valid) begin
            store_input();
        end

        case (state)
            ST_IDLE: begin
                if (conf.ready && conf.valid) begin
                    bit_width <= conf_data.bit_width;
                    bit_width_bpe_mask <= BPE_MASK_SIZE'((1 << conf_data.bit_width) - 1);
                    packed_databeat_bits <= NUM_ELEMENTS * conf_data.bit_width;
                    packed_databeat_bytes <= (NUM_ELEMENTS * conf_data.bit_width) / 8;
                    eight_packed_databeat_bytes <= conf_data.bit_width; // equivalent to (8 * conf_data.bit_width) / 8;
                    rle_width <= (conf_data.bit_width + 7) >> 3;
                    offset <= conf_data.offset;
                    varint_offset <= conf_data.offset;
                    remaining_values <= conf_data.num_values;
                    // Window bases for varint_offset = conf offset under the new configuration.
                    vbase_r <= conf_data.offset + offset_t'(4'((32'(conf_data.bit_width) + 7) >> 3)) + 7'd1;
                    vbase_m[0] <= conf_data.offset + 7'd1;
                    vbase_m[1] <= conf_data.offset + offset_t'(conf_data.bit_width) + 7'd1;
                    vbase_m[2] <= conf_data.offset + offset_t'(2 * conf_data.bit_width) + 7'd1;
                    vbase_m[3] <= conf_data.offset + offset_t'(3 * conf_data.bit_width) + 7'd1;
                    bw_x3 <= offset_t'(3 * conf_data.bit_width);

                    if (in.valid) begin
                        state <= ST_HEADER2;
                        varint_in.data <= win_in;
                        varint_in.valid <= win_in_valid;
                    end else begin
                        state <= ST_HEADER;
                    end
                end
            end

            // Here we have received the configuration but we haven't yet
            // received the input. Receiving one databeat may or may not be
            // enough.
            ST_HEADER: begin
                if (in.valid) begin
                    varint_in.data <= win_in;
                    varint_in.valid <= win_in_valid;
                    // If we receive input, the varint decoding is not yet done
                    // as it takes one cycle. Move to the next state so that
                    // we can optionally take even more input if needed for
                    // the varint decoding.
                    state <= ST_HEADER2;
                end
            end

            // Here we have received the configuration and one input databeat,
            // but we weren't able to decode the header varint with just that.
            ST_HEADER2: begin
                if (varint_out.valid) begin
                    goto_decode(remaining_values);
                end else if (in.valid) begin
                    varint_in.data <= win_next;
                    varint_in.valid <= win_next_valid;
                end
            end

            ST_DECODE_RLE: begin
                if (rle_in.ready && rle_in.valid) begin
                    finish_rle();
                end
            end

            ST_DECODE_BPE: begin
                if (bpe_in.ready && bpe_in.valid) begin
                    if (bpe_in.last) begin
                        finish_bpe();
                    end else begin
                        advance_bpe();
                    end
                end
            end
        endcase
    end
end

// ------- Driving input ---------
always_comb begin
    // We need to provide default values to prevent latch inference
    conf.ready = state == ST_IDLE; 

    in.ready = 0;
    case (state)
        ST_IDLE:
            in.ready = conf.valid;

        ST_HEADER, ST_HEADER2:
            in.ready = ~varint_in.valid;

        ST_DECODE_RLE:
            in.ready = rle_needs_more_input;
            
        ST_DECODE_BPE:
            in.ready = bpe_needs_more_input;
    endcase
end

// ------- Driving output --------
localparam FIFO_DEPTH = MAX_IN_TRANSIT * 8;

logic fifo_out_ready;
logic[$clog2(FIFO_DEPTH):0] filling_level;
// Distributed RAM: the read data is a flop next to the logic it selects (a RAMB18 read was the start of
// the original's worst hybrid path, decoder-timing-floor-01 paths #6-8); a 64 x 1 FIFO fills one LUT pair.
MehdiFIFO #(
    .DEPTH(FIFO_DEPTH),
    .WIDTH($bits(output_t)),
    .STYLE("distributed")
) inst_output_fifo (
    .i_clk(clk),
    .i_rst_n(reset_synced),

    .i_data(next_out.data),
    .i_valid(next_out.valid),
    .i_ready(can_decode_next),

    .o_data(curr_out.data),
    .o_valid(curr_out.valid),
    .o_ready(fifo_out_ready),

    .o_filling_level(filling_level)
);

always_comb begin
    rle_out.ready = 0;
    bpe_out.ready = 0;
    out.valid = 0;
    out.data = '{default: 'x};
    out.keep = '0;
    out.last = 0;
    fifo_out_ready = 0;

    if (curr_out.valid) begin
        case (curr_out.data)
            OUTPUT_RLE: begin
              rle_out.ready = out.ready;
              out.valid = rle_out.valid;
              out.data = rle_out.data;
              out.keep = rle_out.keep;
              out.last = rle_out.last;
              fifo_out_ready = out.ready && out.valid && out.last;
            end

            OUTPUT_BPE: begin
              bpe_out.ready = out.ready;
              out.valid = bpe_out.valid;
              out.data = bpe_out.data;
              out.keep = bpe_out.keep;
              out.last = bpe_out.last;
              fifo_out_ready = out.ready && out.valid;
            end
        endcase
    end
end

// `ifdef SYNTHESIS
// ila_run_decoder inst_ila_run_decoder (
//     .clk(clk),
//     .probe0(reset_synced),
//
//     .probe1(conf.ready),
//     .probe2(conf.valid),
//
//     .probe3(in.ready),
//     .probe4(in.valid),
//     .probe5(in.last),
//
//     .probe6(out.ready),
//     .probe7(out.valid),
//     .probe8(out.last),
//
//     .probe9(state),
//     .probe10(offset),
//     .probe11(varint_offset),
//     .probe12(varint_in.valid),
//     .probe13(varint_in.data),
//     .probe14(varint_out.valid),
//     .probe15(varint_out.data),
//
//     .probe16(remaining_values),
//     .probe17(bpe_count),
//     .probe18(bpe_remaining_inputs),
//     .probe19(rle_width),
//     .probe20(rle_count),
//
//     .probe21(bpe_in.ready),
//     .probe22(bpe_in.valid),
//     .probe23(bpe_in.last),
//
//     .probe24(rle_in.ready),
//     .probe25(rle_in.valid),
//     .probe26(rle_in.last),
//
//     .probe27(bpe_out.ready),
//     .probe28(bpe_out.valid),
//     .probe29(bpe_out.last),
//
//     .probe30(rle_out.ready),
//     .probe31(rle_out.valid),
//     .probe32(rle_out.last),
//
//     .probe33(next_out.valid),
//     .probe34(next_out.data),
//     .probe35(can_decode_next),
//
//     .probe36(curr_out.valid),
//     .probe37(curr_out.data),
//     .probe38(fifo_out_ready),
//
//     .probe39(filling_level)
// );
// `endif

endmodule

`timescale 1ns / 1ps

`include "axi_macros.svh"
`include "lynx_macros.svh"

import lynxTypes::*;
import parcore::*;

interface bpe_stage_i #(
    parameter type input_t,
    parameter type tag_t,
    parameter type data_t,
    parameter NUM_ELEMENTS
);
    input_t raw;
    tag_t   tag;

    data_t[NUM_ELEMENTS - 1:0] data;
    logic[NUM_ELEMENTS - 1:0]  keep;
    logic                      last;
    logic                      valid;
    logic                      ready;

    modport m (
        input  ready,
        output raw, tag, data, keep, last, valid
    );

    modport s (
        input  raw, tag, data, keep, last, valid,
        output ready
    );
endinterface

module ExpandBPE #(
    parameter type data_t,
    parameter NUM_ELEMENTS,
    parameter MAX_IN_TRANSIT = 8
) (
    input logic clk,
    input logic rst_n,

    tagged_i.s in, // #(logic [$bits(data_t) * NUM_ELEMENTS - 1:0], $bits(bpe_config_t))
    ndata_i.m out  // #(data_t, NUM_ELEMENTS)
);

localparam N_STAGES = 4;
localparam int IDX_BOUNDARIES[N_STAGES+1] = '{
  0,
  NUM_ELEMENTS / N_STAGES,
  NUM_ELEMENTS * 2 / N_STAGES,
  NUM_ELEMENTS * 3 / N_STAGES,
  NUM_ELEMENTS
};

typedef logic [$bits(data_t) * NUM_ELEMENTS - 1:0] input_t;

tagged_i #(input_t, $bits(bpe_config_t)) in_inner(clk, rst_n);
bpe_stage_i #(input_t, bpe_config_t, data_t, NUM_ELEMENTS) middle[N_STAGES:0]();

// some stages of buffering are required for full throughput in RunDecoder
// Distributed RAM instead of the default RAMB: 8 entries need 26 RAM32M16 next to the RunDecoder,
// so the write data (RunDecoder's 64:1 byte shift) no longer routes 1.6-1.7 ns to a BRAM column
// (diag-decoder-d0hse-timing-03: data_reg -> s_memory_reg_*/DINADIN, 4 levels, -0.065 ns) and the
// read data feeding stage 0's shift is a flop instead of a RAMB clock-to-out (proxy decoder-hse_r2-01:
// s_memory_reg_0 -> stage data_reg, -0.100 ns). MehdiFIFO's behaviour does not depend on STYLE.
MehdiFIFO #(
    .DEPTH(MAX_IN_TRANSIT),
    .WIDTH($bits(input_t) + $bits(bpe_config_t) + 1 + 1),
    .STYLE("distributed")
) inst_output_fifo (
    .i_clk(clk),
    .i_rst_n(rst_n),

    .i_data({in.data, in.tag, in.keep, in.last}),
    .i_valid(in.valid),
    .i_ready(in.ready),

    .o_data({in_inner.data, in_inner.tag, in_inner.keep, in_inner.last}),
    .o_valid(in_inner.valid),
    .o_ready(in_inner.ready),

    .o_filling_level()
);

assign in_inner.ready = middle[0].ready;
assign middle[0].valid = in_inner.valid;
assign middle[0].raw = in_inner.data;
assign middle[0].tag = in_inner.tag;
assign middle[0].data = 'x;
assign middle[0].keep = 'x;
assign middle[0].last = in_inner.last;

generate
    for (genvar i = 1; i <= N_STAGES; i++) begin : gen_expand_bpe_stages
        ExpandBPEStage #(
            .ID(i-1),
            .data_t(data_t),
            .NUM_ELEMENTS(NUM_ELEMENTS),
            .START_IDX_INCL(IDX_BOUNDARIES[i-1]),
            .END_IDX_EXCL(IDX_BOUNDARIES[i])
        ) inst_expand_bpe_stage (
            .clk(clk),
            .rst_n(rst_n),

            .in(middle[i-1]),
            .out(middle[i])
        );
    end
endgenerate

assign middle[N_STAGES].ready = out.ready;
assign out.valid = middle[N_STAGES].valid;
assign out.data = middle[N_STAGES].data;
assign out.keep = middle[N_STAGES].keep;
assign out.last = middle[N_STAGES].last;

endmodule

module ExpandBPEStage #(
    parameter ID,
    parameter type data_t,
    parameter NUM_ELEMENTS,
    parameter START_IDX_INCL,
    parameter END_IDX_EXCL
) (
    input logic clk,
    input logic rst_n,

    bpe_stage_i.s in,  // #(logic [$bits(data_t) * NUM_ELEMENTS - 1:0], bpe_config_t, data_t, NUM_ELEMENTS)
    bpe_stage_i.m out  // #(logic [$bits(data_t) * NUM_ELEMENTS - 1:0], bpe_config_t, data_t, NUM_ELEMENTS)
);

`ASSERT_ELAB(START_IDX_INCL < END_IDX_EXCL)

assign in.ready = out.ready;

typedef logic [$bits(data_t) * NUM_ELEMENTS - 1:0] input_t;

bpe_stage_i #(input_t, bpe_config_t, data_t, NUM_ELEMENTS) curr(), next();

assign out.raw = curr.raw;
assign out.tag = curr.tag;
assign out.data = curr.data;
assign out.keep = curr.keep;
assign out.last = curr.last;
assign out.valid = curr.valid;

always_comb begin
    next.raw = curr.raw;
    next.tag = curr.tag;
    next.data = curr.data;
    next.keep = curr.keep;
    next.last = curr.last;
    next.valid = curr.valid;

    if (in.valid && out.ready) begin
        next.raw = in.raw;
        next.tag = in.tag;
        next.data = in.data;
        next.keep = in.keep;
        next.last = in.last;
        next.valid = 1;

        for (int unsigned I = START_IDX_INCL; I < END_IDX_EXCL; I++) begin
            next.data[I] = in.raw[I * in.tag.bit_width +: $bits(data_t)] & data_t'(in.tag.mask);
            next.keep[I] = I < in.tag.count;
        end
    end else if (out.ready) begin
        next.raw = 'x;
        next.tag = 'x;
        next.data = 'x;
        next.keep = 'x;
        next.valid = 0;
        next.last = 0;
    end
end

always_ff @(posedge clk) begin
    if (rst_n == 1'b0) begin
        curr.raw <= 'x;
        curr.tag <= 'x;
        curr.data <= 'x;
        curr.keep <= 'x;
        curr.last <= 0;
        curr.valid <= 0;
    end else begin
        curr.raw <= next.raw;
        curr.tag <= next.tag;
        curr.data <= next.data;
        curr.keep <= next.keep;
        curr.last <= next.last;
        curr.valid <= next.valid;
    end
end

endmodule

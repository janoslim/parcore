`timescale 1ns / 1ps

import lynxTypes::*;
import libstf::*;

module VHSNUnzipWrapper #(
    parameter NUM_BYTES = AXI_DATA_BITS / 8,
    parameter DUAL_ISSUE = 1
) (
    input logic clk,
    input logic rst_n,

    ndata_i.s in, // #(data8_t, NUM_BYTES)
    ndata_i.m out // #(data8_t, NUM_BYTES)
);

// Decompressor input paused and reset logic
reg decompressor_input_paused;
reg [1:0] decompressor_reset_counter;

ndata_i #(data8_t, NUM_BYTES) out_inner(clk, rst_n);

VHSNUnzipWrapperInternal #(NUM_BYTES, DUAL_ISSUE) inst_vhsnunzip_wrapper_internal (
    .clk(clk),
    .rst_n(rst_n && decompressor_reset_counter == 3'd0),

    .in(in),
    .out(out_inner)
);

VHSNUnzipWrapperFixLast #(NUM_BYTES) inst_vhsnunzip_wrapper_fix_last (
    .clk(clk),
    .rst_n(rst_n),

    .in(out_inner),
    .out(out)
);

always_ff @(posedge clk) begin
    if (rst_n == 1'b0) begin
        decompressor_input_paused <= 1'b0;
        decompressor_reset_counter <= 0;
    end else begin
        if (in.ready && in.valid && in.last) begin
            decompressor_input_paused <= 1'b1;
        end else if (decompressor_input_paused && out_inner.ready && out_inner.valid && out_inner.last) begin
            decompressor_reset_counter <= 2'd2;
        end else if (decompressor_input_paused && decompressor_reset_counter > 0) begin
            if (decompressor_reset_counter == 2'd1) begin
                decompressor_input_paused <= 1'b0;
            end
            decompressor_reset_counter <= decompressor_reset_counter - 1;
        end
    end
end

endmodule

module VHSNUnzipWrapperInternal #(
    parameter NUM_BYTES = AXI_DATA_BITS / 8,
    parameter DUAL_ISSUE = 1
) (
    input logic clk,
    input logic rst_n,

    ndata_i.s in, // #(data8_t, NUM_BYTES)
    ndata_i.m out // #(data8_t, NUM_BYTES)
);
    // Decompressor parameters. DECOMP_DATA_BYTES must match C_BYTES in
    // vhsnunzip's vhsnunzip_utils_pkg.vhd (the datapath lane width); the cnt
    // widths are derived from it exactly as C_IDX / C_CNT are in the VHDL.
    localparam DECOMP_DATA_BYTES = 16;
    localparam DECOMP_DATA_BITS = DECOMP_DATA_BYTES * 8;
    localparam DECOMP_IN_CNT_BITS = $clog2(DECOMP_DATA_BYTES);
    localparam DECOMP_OUT_CNT_BITS = $clog2(DECOMP_DATA_BYTES) + 1;

    localparam CNT_BITS = $clog2(NUM_BYTES);
    localparam INDEX_BITS = $clog2(NUM_BYTES / DECOMP_DATA_BYTES);

    // Decompressor input ports
    logic co_valid;   
    logic co_ready;
    logic [DECOMP_DATA_BITS-1:0] co_data;
    logic [DECOMP_IN_CNT_BITS-1:0] co_cnt;
    logic co_last;

    // Index in the input chunk
    logic [INDEX_BITS-1:0] in_index;
    // Was the data in the input buffers 'in_*' already processed?
    logic in_done;
    // Number of valid bytes in 'in_data' with implicit MSB
    logic [CNT_BITS - 1:0] in_cnt;

    // Buffers for AXI data
    logic [NUM_BYTES * 8 - 1:0] in_data;
    logic [NUM_BYTES - 1:0] in_keep;
    logic in_last;

    // Decompressor output ports
    logic de_valid;
    logic de_ready;
    logic de_dvalid;
    logic [DECOMP_DATA_BITS-1:0] de_data;
    logic [DECOMP_OUT_CNT_BITS-1:0] de_cnt;
    logic de_last;

    // Index in the output chunk
    logic [INDEX_BITS-1:0] out_index;

    // Ready to receive new input when we are done with the chunk or currently streaming the last part of it.
    // assign in.ready = rst_n && (in_done || (in_index == {(INDEX_BITS){1'b1}} && co_ready));
    always_comb begin
        in.ready = 1'b0;
        if (rst_n) begin
            if (in_done) begin
                in.ready = 1'b1;
            end else if (in_index == {(INDEX_BITS){1'b1}} && co_ready) begin
                in.ready = 1'b1;
            end
        end
    end

    // in_index and in_done
    always_ff @(posedge clk) begin
	    if (rst_n) begin
            if (in.ready) begin
                if (in.valid) begin
                    // Read input chunk
                    for (int i = 0; i < NUM_BYTES; i++) begin
                        in_data[i*8 +: 8] <= in.data[i];
                    end
                    in_keep <= in.keep;
                    in_last <= in.last;

                    in_index <= 0;
                    in_done <= 1'b0;
                end else begin
                    // Stop supplying data if there is no new input
                    in_done <= 1'b1;
                end
            end else if (co_ready && co_valid) begin
                if (co_last) begin
                    // Internal reset
                    in_index <= 0;
                    in_done <= 1'b1;
                end else begin
                    in_index <= in_index + 1;
                end
            end
    	end else begin
  	        in_index <= 0;
            in_done <= 1'b1;
        end
    end

    // in_cnt
    always_comb begin
        in_cnt = 0;
        for (int i = 0, done = 0; i < AXI_DATA_BITS/8; i++) begin
            if (!done && !in_keep[i]) begin
                in_cnt = i;
                done = 1;
            end
        end
    end

    // co_data, co_cnt, co_last and co_valid
    always_comb begin
        co_data = in_data[in_index*DECOMP_DATA_BITS+:DECOMP_DATA_BITS];

        co_cnt = 0;
        co_last = 1'b0;
        if (0 < in_cnt && in_cnt <= (in_index + 1) * DECOMP_DATA_BITS/8) begin
            // Section fed to the decompressor is only partially valid
            co_cnt = in_cnt[DECOMP_IN_CNT_BITS-1:0];
            co_last = 1'b1;
        end else if (in_last && in_index == {INDEX_BITS{1'b1}}) begin
            // Input is last chunk and index is at the end
            co_last = 1'b1;
        end

        co_valid = rst_n && !in_done;
    end
    
    // We can accept new decompressed data if the buffer 'out.data' is not full yet
    // or if we are in an output handshake and the buffer will be empty next cycle.
    // assign de_ready = rst_n && (!out.valid || out.ready);
    always_comb begin
        de_ready = 1'b0;
        if (rst_n) begin
            if (!out.valid) begin
                de_ready = 1'b1;
            end else if (out.ready) begin
                de_ready = 1'b1;
            end
        end
    end

    // out_index
    always_ff @(posedge clk) begin
        if (rst_n) begin
            if (de_ready && de_valid) begin
                if (de_last) begin
                    // Internal reset
                    out_index <= 0;
                end else begin
                    out_index <= out_index + 1;
                end
            end 
        end else begin
            out_index <= 0;
        end
    end

    // out.data, out.keep and out.last
    always_ff @(posedge clk) begin
        logic [AXI_DATA_BITS/8-1:0] tmp_keep;
        if (rst_n) begin
            if (de_ready && de_valid) begin
                // Read decompressed data
                for (int i = 0; i < DECOMP_DATA_BYTES; i++) begin
                    out.data[out_index*DECOMP_DATA_BYTES + i] <= de_data[i*8 +: 8];
                end
                // out.data[out_index*DECOMP_DATA_BITS+:DECOMP_DATA_BITS] <= de_data;
                out.last <= de_last;
            end

            tmp_keep = out.keep;
            if (out.ready && out.valid) begin
                // Reset keep signal on output handshake.
                tmp_keep = 0;
            end
            if (de_ready && de_valid) begin
                // Compute keep signal for read data.
                tmp_keep[out_index*DECOMP_DATA_BITS/8+:DECOMP_DATA_BITS/8] =
                    de_dvalid
                        ? (de_cnt == 0 
                            ? {(DECOMP_DATA_BITS/8){1'b1}}
                            : (1 << de_cnt) - 1)
                        : 0;
            end
            out.keep <= tmp_keep;
        end else begin
            // A short first page never writes the unused upper keep slices.
            out.keep <= '0;
            out.last <= 1'b0;
        end
    end

    // out.valid
    always_ff @(posedge clk) begin
        if (rst_n) begin
            if (de_ready && de_valid && (out_index == {INDEX_BITS{1'b1}} || de_last)) begin
                // We are ready to transmit the next chunk after the final (for the chunk or in total) bit was read.
                out.valid <= 1'b1;
            end else if (out.valid && out.ready) begin
                // Reset after handshake.
                out.valid <= 1'b0;
            end
        end else begin
            out.valid <= 1'b0;
        end
    end

    vhsnunzip_unbuffered #(
        .LONG_CHUNKS(1),
        .DUAL_ISSUE(DUAL_ISSUE),
        .RAM_STYLE("ultra")
    ) decompressor (
        .clk(clk),
        .reset(~rst_n),

        .co_valid(co_valid),
        .co_ready(co_ready),
        .co_data(co_data),
        .co_cnt(co_cnt),
        .co_last(co_last),

        .de_valid(de_valid),
        .de_ready(de_ready),
        .de_dvalid(de_dvalid),
        .de_data(de_data),
        .de_cnt(de_cnt),
        .de_last(de_last)
    );
endmodule

module VHSNUnzipWrapperFixLast #(
    parameter NUM_BYTES = AXI_DATA_BITS / 8
) (
    input logic clk,
    input logic rst_n,

    ndata_i.s in, // #(data8_t, NUM_BYTES)
    ndata_i.m out // #(data8_t, NUM_BYTES)
);

ndata_i #(data8_t, NUM_BYTES) tmp(clk, rst_n);
logic is_ghost;
// A "ghost" databeat is a valid beat which carries no data but the last
// signal.
assign is_ghost = in.last && in.keep == '0;

logic consuming_in;
assign consuming_in = in.ready && in.valid;

always_ff @(posedge clk) begin
    if (!rst_n) begin
        tmp.valid  <= 1'b0;
    end else begin
        if (consuming_in && ~is_ghost) begin
            tmp.valid <= in.valid;
            tmp.data <= in.data;
            tmp.keep <= in.keep;
            tmp.last <= in.last;
        end else if(out.ready && out.valid) begin
            tmp.valid <= 1'b0;
        end
    end
end

// Assign ready to silence assertion that ready cannot be undefined. Needs to be high so we do not 
// get in trouble with with stable assertion of the interface.
assign tmp.ready = 1'b1;

assign out.valid = tmp.valid && (tmp.last || in.valid);
assign out.data  = tmp.data;
assign out.keep  = tmp.keep;
assign out.last  = tmp.last || is_ghost;

assign in.ready = ~tmp.valid || out.ready;

endmodule

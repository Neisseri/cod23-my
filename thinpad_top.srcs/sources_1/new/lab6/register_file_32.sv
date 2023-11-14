`default_nettype none

module register_file_32 (
    // 32 regs, 32 bit-width
    input wire clk,
    input wire reset,

    input wire [4 : 0] waddr, // writing address
    input wire [31 : 0] wdata, // writing data

    input wire we, // enable signal of writing

    input wire [ 4:0] raddr_a, // register id of reading port 'a'
    output reg [31:0] rdata_a, // register data of reading port 'a'
    input wire [ 4:0] raddr_b, // register id of reading port 'b'
    output reg [31:0] rdata_b // register data of reading port 'b'
);

    reg[UNIT_SIZE - 1 : 0] mem[SIZE];

    // writing port
    always_ff @(posedge clk) begin
        if (reset) begin
            for (integer i = 0; i < 32; i++) begin
                mem[i] <= 16'b0;
            end
            rdata_a <= 16'b0;
            rdata_b <= 16'b0;
        end else begin
            rdata_a <= mem[raddr_a];
            rdata_b <= mem[raddr_b];
            if (we == 1'b1 && waddr != 0) begin
                mem[waddr] <= wdata;
            end
        end
    end

endmodule


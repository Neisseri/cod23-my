module register_file #(
    parameter UNIT_SIZE = 16, // bit width
    parameter SIZE = 32 // register number
)(
    input wire clk,

    input wire [$clog2(SIZE) - 1 : 0] waddr, // writing addresss
    input wire [UNIT_SIZE - 1 : 0] wdata, // writing data

    input wire we, // enable signal of writing

    input wire [$clog2(SIZE) - 1 : 0] raddr_a, // register id of reading port 'a'
    output reg [UNIT_SIZE - 1 : 0] rdata_a, // register data of reading port 'a'
    input wire [$clog2(SIZE) - 1 : 0] raddr_b, // register id of reading port 'b'
    output reg [UNIT_SIZE - 1 : 0] rdata_b // register data of reading port 'b'
);

    reg[UNIT_SIZE - 1 : 0] mem[SIZE];

    // reading port a
    always_ff @(raddr_a) begin
        rdata_a <= mem[raddr_a]; 
    end

    // reading port b
    always_ff @(raddr_b) begin
        rdata_b <= mem[raddr_b];
    end

    // writing port
    always_ff @(posedge clk) begin
        if (we == 1'b1 && waddr != 0) begin
            mem[waddr] <= wdata;
        end
    end

endmodule


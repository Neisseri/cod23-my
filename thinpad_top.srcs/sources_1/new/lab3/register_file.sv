module register_file #(
    parameter UNIT_SIZE = 8,
    parameter SIZE = 1024
)(
    input wire [$clog2(SIZE) - 1 : 0] raddr_i,
    output wire [UNIT_SIZE - 1 : 0] rdata_o
);

    reg[UNIT_SIZE - 1 : 0] mem[SIZE];

endmodule
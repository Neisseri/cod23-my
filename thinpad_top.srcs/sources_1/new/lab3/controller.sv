`default_nettype none

module controller (
    input wire clk,
    input wire reset,

    // signals connected to register file module
    output reg [ 4:0] rf_raddr_a, // register id of reading port a
    input wire [15:0] rf_rdata_a = 16'b0, // register data of reading port a
    output reg [ 4:0] rf_raddr_b,
    input wire [15:0] rf_rdata_b = 16'b0,

    output reg [ 4:0] rf_waddr, // register address of writing
    output reg [15:0] rf_wdata, // register data of writing
    output reg rf_we, // enable signal of writing

    // signal connected to ALU module
    output reg [15:0] alu_a,
    output reg [15:0] alu_b,
    output reg [ 3:0] alu_op,
    input wire [15:0] alu_y = 16'b0,

    // controlling signal
    input wire step = 0, // user button state pulse
    input wire [31:0] dip_sw = 32'b0, // 32 bits dip switch
    output reg [15:0] leds
);

    logic [31:0] inst_reg; // instruction register

    // combinatorial logic
    // resolve common parts of instructions
    // depending on valid 'inst_reg' value
    logic is_rtype, is_itype, is_peek, is_poke;
    logic [15:0] imm; // immediate number
    logic [ 4:0] rd, rs1, rs2;
    logic [ 3:0] opcode;

    always_comb begin // combinatorial logic
        is_rtype = (inst_reg[2:0] == 3'b001);
        is_itype = (inst_reg[2:0] == 3'b010);
        is_peek = is_itype && (inst_reg[6:3] == 4'b0010);
        is_poke = is_itype && (inst_reg[6:3] == 4'b0001);

        imm = inst_reg[31:16];
        rd = inst_reg[11:7];
        rs1 = inst_reg[19:15];
        rs2 = inst_reg[24:20];
        opcode = inst_reg[6:3];
    end

    // define state list with Enum
    // datatype is 'logic [3:0]'
    typedef enum logic [3:0] {
        ST_INIT,
        ST_DECODE,
        ST_CALC,
        ST_READ_REG,
        ST_WRITE_REG,
        ST_WAIT,
        ST_WAIT_CALC,
        ST_WAIT_READ
    } state_t;

    // current state register of state machine
    state_t state;

    // state machine logic
    always_ff @(posedge clk) begin // temporal logic
        if (reset) begin
            // TODO: reset output signals
            rf_raddr_a <= 5'b0;
            rf_raddr_b <= 5'b0;
            
            rf_waddr <= 5'b0;
            rf_wdata <= 16'b0;
            rf_we <= 1'b0;

            alu_a <= 16'b0;
            alu_b <= 16'b0;
            alu_op <= 4'b0;
            leds <= 16'b0;

            state <= ST_INIT;
        end else begin
            case (state)
                ST_INIT: begin
                    rf_we <= 1'b0;
                    if (step) begin
                        inst_reg <= dip_sw;
                        state <= ST_DECODE;
                    end
                end

                ST_DECODE: begin
                    if (is_rtype) begin
                        // transfer register address to register file
                        // read in operand
                        rf_raddr_a <= rs1;
                        rf_raddr_b <= rs2;
                        state <= ST_WAIT;
                    end else if (is_itype) begin
                        // TODO: other instructions
                        if (is_peek) begin // read data from rd
                            rf_raddr_a <= rd;
                            state <= ST_WAIT_READ;
                        end else if (is_poke) begin // write imm into rd
                            rf_waddr <= rd;
                            rf_wdata <= imm;
                            rf_we <= 1'b1;
                            state <= ST_WRITE_REG;
                        end
                    end else begin
                        // undefined instruction
                        // return to initial state
                        state <= ST_INIT;
                    end
                end

                ST_WAIT: begin
                    state <= ST_CALC;
                end

                ST_CALC: begin
                    // TODO: transfer data to ALU
                    // get results from ALU
                    alu_a <= rf_rdata_a;
                    alu_b <= rf_rdata_b;
                    alu_op <= opcode;
                    state <= ST_WAIT_CALC;
                end

                ST_WAIT_CALC: begin
                    rf_waddr <= rd;
                    rf_wdata <= alu_y;
                    rf_we <= 1'b1;
                    state <= ST_WRITE_REG;
                end

                ST_WRITE_REG: begin
                    // TODO: store results in register
                    
                    state <= ST_INIT;
                end

                ST_WAIT_READ: begin
                    state <= ST_READ_REG;
                end

                ST_READ_REG: begin
                    // TODO: read data from register
                    // store into 'leds'
                    leds <= rf_rdata_a;
                    state <= ST_INIT;
                end

                default: begin
                    state <= ST_INIT;
                end

            endcase
        end
    end
endmodule
`default_nettype none

module alu_32 (
    input  wire signed [31:0] a = 32'b0,
    input  wire signed [31:0] b = 32'b0, // input operand
    input  wire [ 3:0] op = 4'b0, // operator
    output reg  signed [31:0] y = 32'b0// output data
);

    always_comb begin
        case (op)
            4'b0001: begin // 1: ADD
                y = a + b;
            end

            4'b0010: begin // 2: SUB
                y = a - b;
            end

            4'b0011: begin // 3: AND
                y = a & b;
            end

            4'b0100: begin // 4: OR
                y = a | b;
            end

            4'b0101: begin // 5: XOR
                y = a ^ b;
            end

            4'b0110: begin // 6: NOT
                y = ~ a;
            end

            4'b0111: begin // 7: SLL
                y = a << b[3:0];
            end

            4'b1000: begin // 8: SRL
                y = a >> b[3:0];
            end

            4'b1001: begin // 9: SRA
                y = a >>> b[3:0];
            end

            // TODO: 关于移动位数的问题可能需要修改
            4'b1010: begin // 10: ROL
                case (b[3:0])
                    
                    4'b0000: begin // 0
                        y = a;
                    end

                    4'b0001: begin // 1
                        y = {a[14:0], a[15]};
                    end

                    4'b0010: begin // 2
                        y = {a[13:0], a[15:14]};
                    end

                    4'b0011: begin // 3
                        y = {a[12:0], a[15:13]};
                    end

                    4'b0100: begin // 4
                        y = {a[11:0], a[15:12]};
                    end

                    4'b0101: begin // 5
                        y = {a[10:0], a[15:11]};
                    end

                    4'b0110: begin // 6
                        y = {a[9:0], a[15:10]};
                    end

                    4'b0111: begin // 7
                        y = {a[8:0], a[15:9]};
                    end

                    4'b1000: begin // 8
                        y = {a[7:0], a[15:8]};
                    end

                    4'b1001: begin // 9
                        y = {a[6:0], a[15:7]};
                    end

                    4'b1010: begin // 10
                        y = {a[5:0], a[15:6]};
                    end

                    4'b1011: begin // 11
                        y = {a[4:0], a[15:5]};
                    end

                    4'b1100: begin // 12
                        y = {a[3:0], a[15:4]};
                    end

                    4'b1101: begin // 13
                        y = {a[2:0], a[15:3]};
                    end

                    4'b1110: begin // 14
                        y = {a[1:0], a[15:2]};
                    end

                    4'b1111: begin // 15
                        y = {a[0], a[15:1]};
                    end

                    default: begin
                        y = a;
                    end

                endcase
            end

            default: begin
                y = 16'b0;
            end
        endcase
    end
endmodule
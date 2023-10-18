module alu (
    input  wire [15:0] a;
    input  wire [15:0] b; // input operand
    input  wire [ 3:0] op; // operator
    output wire [15:0] y; // output data
);

    always_comb begin
        case (op)
            4'b0001: begin // 1: ADD
                y <= a + b;
            end

            4'b0010: begin // 2: SUB
                y <= a - b;
            end

            4'b0011: begin // 3: AND
                y <= a & b;
            end

            4'b0100: begin // 4: OR
                y <= a | b;
            end

            4'b0101: begin // 5: XOR
                y <= a ^ b;
            end

            4'b0110: begin // 6: NOT
                y <= ~ a;
            end

            4'b0111: begin // 7: SLL
                y <= a << b;
            end

            4'b1000: begin // 8: SRL
                y <= $signed(a) >> b;
            end

            4'b1001: begin // 9: SRA
                y <= a >>> b;
            end

            4'b1010: begin // 10: ROL
                y <= a  b;
                y <= {a[b - 1 : 0], a[15 : b]};
            end

            default: begin
            end
        endcase
    end
endmodule
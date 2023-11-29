`default_nettype none
// Empty top module

module top (
  // I/O ports
  // Specific to FPGA modeled on (https://verilog.ecn.purdue.edu/) - a verilog simulator created by course coordinator Niraj Menon
  input  logic hz100, reset,
  input  logic [20:0] pb,
  output logic [7:0] left, right,
         ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0,
  output logic red, green, blue,

  // UART ports
  output logic [7:0] txdata,
  input  logic [7:0] rxdata,
  output logic txclk, rxclk,
  input  logic txready, rxready
);

// 4Hz Clock
logic hz4;
assign right[0] = hz4;
clock_4hz ck(.clk(hz100), .rst(reset), .hz4(hz4));

// Lunar Lander instantiation
lunarlander #(16'h800, 16'h4500, 16'h0, 16'h5) ll (
        .clk100(hz100), .clk(hz4), .reset(reset), .in(pb[19:0]), .fail(red), .land(green),
        .seg({ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0})
      );
    
endmodule

module lunarlander #(parameter Fuel = 16'h800, parameter Alt = 16'h4500, parameter Vel = 16'h0, parameter Thrust = 16'h5,
                     parameter GRAV = 16'h5) (input logic clk100, clk, reset, input logic [19:0] in, output logic fail, land,
                     output logic [63:0] seg);

    // Logic Initialization
    logic [15:0] alt, alt_2, v, v_2, fuel, fuel_2, thrust, thrust_man, carry;
    logic pulse;
    logic [4:0] res;
    logic [1:0] select;
    logic [23:0] lookupmsg [3:0];
    logic [15:0] val;
    logic [15:0] negval;
    logic [63:0] valdisp, negvaldisp;

    // Calculations
    bcdaddsub4 u1(.a(alt), .b(v), .op(0), .s(alt_2));
    bcdaddsub4 u2(.a(v), .b(GRAV), .op(1), .s(carry));
    bcdaddsub4 u3(.a(carry), .b(thrust), .op(0), .s(v_2));
    bcdaddsub4 u4_(.a(fuel), .b(thrust), .op(1), .s(fuel_2));

    // Thrust
    scankey s1(.clk(clk100), .rst(reset), .in(in), .out(res), .strobe(pulse));

    always_ff @ (posedge pulse, posedge reset) begin
        if (reset == 1'b1) begin
            thrust_man <= Thrust;
        end
        else if (res <= 9) begin
            thrust_man <= {12'b000000000000, res[3:0]};
        end
    end

    // State Machine
    typedef enum logic [2:0] {INIT=0, CALC=1, SET=2, CHK=3, HLT=4} flight_t;
        logic [2:0] flight;
        logic nland, ncrash;

    always_ff @ (posedge clk, posedge reset) begin
        if (reset == 1'b1) begin
            ncrash <= 1'b0;
            nland <= 1'b0;
            fail <= 1'b0;
            land <= 1'b0;
            flight <= INIT;
            fuel <= Fuel;
            alt <= Alt;
            v <= Vel;
            thrust <= Thrust;
        end
        else begin
            if (flight == INIT) begin
                flight <= CALC;
            end
            else if (flight == CALC) begin 
                flight <= SET;
            end
            else if (flight == SET) begin
                if (fuel_2[15] == 1'b1) begin
                    fuel <= 16'd0;
                end
                else begin
                    fuel <= fuel_2;
                end
                alt <= alt_2;
                v <= v_2;
                if ((fuel == 16'd0) | (fuel_2[15] == 1'b1)) begin
                    thrust <= 16'd0;
                end
                else begin
                    thrust <= thrust_man;
                end
                flight <= CHK;
            end

            else if (flight == CHK) begin
                if ((v_2 > 16'h9970) && (alt_2[15] == 1'b1) && (thrust <= 5)) begin
                    flight <= HLT;
                    nland <= 1'b1;
                end
                else if (alt_2[15] == 1'b1) begin
                    flight <= HLT;
                    ncrash <= 1'b1;
                end
                else begin
                    flight <= CALC;
                end
            end

            else begin
                alt <= 16'd0;
                v <= 16'd0;
                land <= nland;
                fail <= ncrash;
            end
        end
    end

    always_comb begin
        lookupmsg[0] = 24'b011101110011100001111000;
        lookupmsg[1] = 24'b001111100111100100111000;
        lookupmsg[2] = 24'b011011110111011101101101;
        lookupmsg[3] = 24'b011110000111011001010000;

        case (select)
        2'b00: val = alt;
        2'b01: val = v;
        2'b10: val = fuel;
        2'b11: val = thrust;
        endcase

    end

    bcdaddsub4 u4(.a(16'd0), .b(val), .op(1'b1), .s(negval));
    display_32_bit d1(.in({16'b0, val}), .out(valdisp));
    display_32_bit d2(.in({16'b0, negval}), .out(negvaldisp));

    always_comb begin
        if (val[15] == 1'b1) begin
            seg = {lookupmsg[select], 8'b00000000, 8'b01000000, negvaldisp[23:0]};
        end
        else begin 
            seg = {lookupmsg[select], 8'b00000000, valdisp[31:0]};
        end
    end

    always_ff @ (posedge pulse, posedge reset) begin
        if (reset == 1'b1) begin
            select <= 2'b0;
        end
        else begin
            if (res == 5'd16) begin
                select <= 2'b11;
            end
            else if (res == 5'd17) begin
                select <= 2'b10;
            end
            else if (res == 5'd18) begin
                select <= 2'b01;
            end
            else if (res == 5'd19) begin
                select <= 2'b00;
            end
        end
    end

endmodule
// Half Adder
module ha (input logic a, b, output logic s, co);
    assign s = a ^ b;
    assign co = a & b;

endmodule
// Full Adder 1-bit
module fa (input logic a, b, ci, output logic s, co);
    assign s = (a ^ b ^ ci);
    assign co = (a & b) | (a & ci) | (b & ci);

endmodule
// Full Adder 4-bit
module fa4 (input logic [3:0] a, b, input logic ci, output logic [3:0] s, output logic co);
    logic co_f1, co_f2, co_f3, co_f4;

    fa f1(.a(a[0]), .b(b[0]), .ci(ci), .s(s[0]), .co(co_f1));
    fa f2(.a(a[1]), .b(b[1]), .ci(co_f1), .s(s[1]), .co(co_f2));
    fa f3(.a(a[2]), .b(b[2]), .ci(co_f2), .s(s[2]), .co(co_f3));
    fa f4(.a(a[3]), .b(b[3]), .ci(co_f3), .s(s[3]), .co(co_f4));
    
    assign co = co_f4;

endmodule
// Full Adder 8-bit
module fa8(input logic [7:0] a, b, input logic ci, output logic [7:0] s, output logic co);
    logic co_f1, co_f2;

    fa4 f1(.a(a[3:0]), .b(b[3:0]), .ci(ci), .s(s[3:0]), .co(co_f1));
    fa4 f2(.a(a[7:4]), .b(b[7:4]), .ci(co_f1), .s(s[7:4]), .co(co_f2));

    assign co = co_f2;

endmodule 

module addsub8 (input logic [7:0] a, b, input logic op, output logic [7:0] s, output logic co);
    logic [7:0] radix;
    assign radix = (b ^ {op, op, op, op, op, op, op, op});

    fa8 f1(.a(a), .b(radix), .ci(op), .s(s), .co(co));

endmodule

module bcdadd1 (input logic [3:0] a, b, input logic ci, output logic co, output logic [3:0] s);
    logic [3:0] y, x;
    logic co_f1, correction;

    fa4 f1(.a(a), .b(b), .ci(ci), .s(y), .co(co_f1));
    
    assign correction = co_f1 | (y[3] & y[2]) | (y[3] & y[1]);
    assign x[3] = 1'b0;
    assign x[0] = 1'b0;
    assign x[1] = correction;
    assign x[2] = correction;

    fa4 f2(.a(x), .b(y), .ci(1'b0), .s(s), .co(co));

endmodule

module bcdadd4 (input logic [15:0] a, b, input logic ci, output logic co, output logic [15:0] s); 
    logic co_f1, co_f2, co_f3, co_f4;


    fa4 f1(.a(a[3:0]), .b(b[3:0]), .ci(ci), .s(s[3:0]), .co(co_f1));
    fa4 f2(.a(a[7:4]), .b(b[7:4]), .ci(co_f1), .s(s[7:4]), .co(co_f2));
    fa4 f3(.a(a[11:8]), .b(b[11:8]), .ci(co_f2), .s(s[11:8]), .co(co_f3));
    fa4 f4(.a(a[15:12]), .b(b[15:12]), .ci(co_f3), .s(s[15:12]), .co(co_f4));
    
    assign co = co_f4;

endmodule

module bcd9comp1 (input logic [3:0] in, output logic [3:0] out);
  always_comb
    case(in) 
      4'd9 : out = 4'd0;
      4'd8 : out = 4'd1;
      4'd7 : out = 4'd2;
      4'd6 : out = 4'd3;
      4'd5 : out = 4'd4;
      4'd4 : out = 4'd5;
      4'd3 : out = 4'd6;
      4'd2 : out = 4'd7;
      4'd1 : out = 4'd8;
      4'd0 : out = 4'd9;
      default: out = 4'd0;
    endcase

endmodule

module bcdaddsub4 (input logic [15:0] a, b, input logic op, output logic [15:0] s);
  logic carry_in;
  logic [15:0] nine_cmp, b_mod;
  // 9 Cpmplements
  bcd9comp1 cmp1(.in(b[3:0]), .out(nine_cmp[3:0]));
  bcd9comp1 cmp2(.in(b[7:4]), .out(nine_cmp[7:4]));
  bcd9comp1 cmp3(.in(b[11:8]), .out(nine_cmp[11:8]));
  bcd9comp1 cmp4(.in(b[15:12]), .out(nine_cmp[15:12]));

  always @* begin
    case (op)
      1'b1 : b_mod = nine_cmp;
      1'b0 : b_mod = b;
      endcase
  end
  
  bcdadd4 ba1(.a(a), .b(b_mod), .ci(op), .s(s));

endmodule

module ssdec (input logic [3:0] in, input logic enable, output logic [6:0] out);

  logic [6:0] SEG7 [15:0];

  assign SEG7[4'h0] = 7'b0111111;
  assign SEG7[4'h1] = 7'b0000110;
  assign SEG7[4'h2] = 7'b1011011;
  assign SEG7[4'h3] = 7'b1001111;
  assign SEG7[4'h4] = 7'b1100110;
  assign SEG7[4'h5] = 7'b1101101;
  assign SEG7[4'h6] = 7'b1111101;
  assign SEG7[4'h7] = 7'b0000111;
  assign SEG7[4'h8] = 7'b1111111;
  assign SEG7[4'h9] = 7'b1100111;
  assign SEG7[4'ha] = 7'b1110111;
  assign SEG7[4'hb] = 7'b1111100;
  assign SEG7[4'hc] = 7'b0111001;
  assign SEG7[4'hd] = 7'b1011110;
  assign SEG7[4'he] = 7'b1111001;
  assign SEG7[4'hf] = 7'b1110001;

  assign out[6:0] = enable ? SEG7[in] : 7'b0000000;

endmodule

module  scankey(input logic clk, rst, input logic [19:0] in, output logic [4:0] out, output logic strobe);

  logic keyclk;
  assign keyclk = |in[19:0];
  
  logic [1:0] delay;
  
  always_ff @ (posedge clk) begin
  if (rst == 1'b1)
    delay[1] <= 0;
  else
    delay <= (delay << 1) | {1'b0, keyclk};
  end
    
  assign strobe = delay[1];
    
  assign out[0] = in[1] | in[3] | in[5] | in[7] | in[9] | in[11] | in[13] | in[15] | in[17] | in[19];
  assign out[1] = in[2] | in[3] | in[6] | in[7] | in[10] | in[11] | in[14] | in[15] | in[18] | in[19];
  assign out[2] = (| in[7:4]) | (| in[15:12]);
  assign out[3] = | in[15:8];
  assign out[4] = | in[19:16];

endmodule

module display_32_bit (input logic [31:0] in, output logic [63:0] out);
        ssdec s0(.in(in[3:0]), .enable(1'b1), .out(out[6:0]));
        ssdec s1(.in(in[7:4]), .enable(|in[31:4]), .out(out[14:8]));
        ssdec s2(.in(in[11:8]), .enable(|in[31:8]), .out(out[22:16]));
        ssdec s3(.in(in[15:12]), .enable(|in[31:12]), .out(out[30:24]));
        ssdec s4(.in(in[19:16]), .enable(|in[31:16]), .out(out[38:32]));
        ssdec s5(.in(in[23:20]), .enable(|in[31:20]), .out(out[46:40]));
        ssdec s6(.in(in[27:24]), .enable(|in[31:24]), .out(out[54:48]));
        ssdec s7(.in(in[31:28]), .enable(|in[31:28]), .out(out[62:56]));
endmodule

module clock_4hz(input logic clk, rst, output logic hz4);

    logic [7:0] ctr;
    logic flash;
    
    count8du c8_1(.CLK(clk), .RST(rst), .DIR(1'b0), .E(1'b1), .MAX(8'd12), .Q(ctr[7:0]));

    always_ff @ (posedge clk) begin
    if (rst == 1'b1)
        flash <= 1'b0;
    else
        flash <= (ctr == 8'd12);
    end

    always_ff @ (posedge flash) begin
    if (rst == 1'b1)
        hz4 <= 1'b0;
    else
        hz4 <= ~hz4;
  end

endmodule

module  count8du(input logic CLK, RST, DIR, E, input logic [7:0] MAX, output logic [7:0] Q);

    logic [7:0] next_Q;


        always_ff @ (posedge CLK, posedge RST) begin
            if (RST == 1'b1)
                Q <= 8'b00000000;
            else  
                Q <= next_Q;
        end

        always_comb begin
          if (E == 1'b1) begin
            if (DIR == 1'b0) begin 
                if (Q == 8'b0) begin
                    next_Q = MAX;
                end
                else begin 
                    next_Q[0] = ~Q[0];
                    next_Q[1] = Q[1] ^ ~Q[0];
                    next_Q[2] = Q[2] ^ (&(~Q[1:0]));
                    next_Q[3] = Q[3] ^ (&(~Q[2:0]));
                    next_Q[4] = Q[4] ^ (&(~Q[3:0]));
                    next_Q[5] = Q[5] ^ (&(~Q[4:0]));
                    next_Q[6] = Q[6] ^ (&(~Q[5:0]));
                    next_Q[7] = Q[7] ^ (&(~Q[6:0]));
                end
            end
            else begin
                if (Q == MAX) begin
                    next_Q = 8'b0;
                end
                else begin
                    next_Q[0] = ~Q[0];
                    next_Q[1] = Q[1] ^ Q[0];
                    next_Q[2] = Q[2] ^ (&Q[1:0]);
                    next_Q[3] = Q[3] ^ (&Q[2:0]);
                    next_Q[4] = Q[4] ^ (&Q[3:0]);
                    next_Q[5] = Q[5] ^ (&Q[4:0]);
                    next_Q[6] = Q[6] ^ (&Q[5:0]);
                    next_Q[7] = Q[7] ^ (&Q[6:0]);
                end
            end
          end
          else begin
            next_Q = Q;
          end     
        end
      
endmodule
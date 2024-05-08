`default_nettype none
// Empty top module

module top (
    // I/O ports
    // Specific to FPGA modeled on (https://verilog.ecn.purdue.edu/) - a verilog simulator created by course coordinator Niraj Menon
    input  logic hz100, reset,
    input  logic [19:0] pb,
    output logic [7:0] left, right,
           ss7, ss6, ss5, ss4, ss3, ss2, ss1, ss0,
    output logic red, green, blue,

    // UART ports
    output logic [7:0] txdata,
    input  logic [7:0] rxdata,
    output logic txclk, rxclk,
    input  logic txready, rxready
    );

    lunarlander LL1 (.hz100(hz100), .reset(reset), .in(pb), .ss7(ss7), .ss6(ss6), .ss5(ss5), .ss3(ss3),
                     .ss2(ss2), .ss1(ss1), .ss0(ss0), .red(red), .green(green));
    
endmodule

module ll_memory 
#(parameter ALTITUDE = 16'h4500, VELOCITY = 16'h0, FUEL = 16'h800, THRUST = 16'h5)
(
    input logic clk, 
    input logic rst, 
    input logic wen, 
    input logic [15:0] alt_n, 
    input logic [15:0] vel_n,
    input logic [15:0] fuel_n, 
    input logic [15:0] thrust_n, 
    output logic [15:0] alt, 
    output logic [15:0] vel,
    output logic [15:0] fuel, 
    output logic [15:0] thrust
);
    // Intermediate Registers
    logic [15:0] current_alt;
    logic [15:0] current_vel;
    logic [15:0] current_fuel;
    logic [15:0] current_thrust;

    // Flip Flop
    always_ff @(posedge clk or posedge rst) begin : FF
        if (rst) begin
            current_alt <= ALTITUDE;
            current_vel <= VELOCITY;
            current_fuel <= FUEL;
            current_thrust <= THRUST;
        end else if (wen) begin
            current_alt <= alt_n;
            current_vel <= vel_n;
            current_fuel <= fuel_n;
            current_thrust <= thrust_n;
        end
    end

    assign alt = current_alt;
    assign vel = current_vel;
    assign fuel = current_fuel;
    assign thrust = current_thrust;

endmodule

module ll_alu 
#(parameter GRAVITY = 16'h5)
(
    input logic [15:0] alt, 
    input logic [15:0] vel, 
    input logic [15:0] fuel, 
    input logic [15:0] thrust,
    output logic [15:0] alt_n, 
    output logic [15:0] vel_n, 
    output logic [15:0] fuel_n
);
    // Intermediaet Registers
    logic [15:0] alt_c;
    logic [15:0] vel_c;
    logic [15:0] vel_c1;
    logic [15:0] fuel_c;

    // Arithmetic 
    bcdaddsub4 alt_add_sub(.a(alt), .b(vel), .op(0), .s(alt_c));
    bcdaddsub4 vel_subtract(.a(vel), .b(GRAVITY),.op(1), .s(vel_c));
    bcdaddsub4 vel_subtract2(.a(vel_c), .b({(fuel == 0) ? 16'h0 :thrust}), .op(0), .s(vel_c1));
    bcdaddsub4 fuel_subtract(.a(fuel), .b(thrust), .op(1), .s(fuel_c));

    // Non-Positive Altitude and Velocity Logic
    always_comb begin
        if (alt_c[15] || alt_c == 0) begin
            alt_n = 16'h0;
            vel_n = 16'h0;
        end else begin
            alt_n = alt_c;
            vel_n = vel_c1;
        end
        fuel_n = (fuel_c[15] || fuel_c == 0) ? 16'h0 : fuel_c;
    end

endmodule

module ll_control (
    input logic clk, 
    input logic rst, 
    input logic [15:0] alt, 
    input logic [15:0] vel, 
    output logic land, 
    output logic crash, 
    output logic wen
    );
    // Intermediate Registers
    logic [15:0] alt_vel_sum;

    // Arithmetic
    bcdaddsub4 alt_vel_add(.a(alt), .b(vel), .op(0), .s(alt_vel_sum));

    // Flip Flop
    always_ff @(posedge clk or posedge rst) begin : FF
        if (rst) begin
            land <= 1'b0;
            crash <= 1'b0;
            wen <= 1'b0;
        end else begin
            if (alt_vel_sum[15] || alt_vel_sum == 0) begin
                if (vel < 16'h9970) begin
                    crash <= 1'b1;
                    land <= 1'b0;
                    wen <= 1'b0;
                end
                else begin
                    crash <= 1'b0;
                    land <= 1'b1;
                    wen <= 1'b0;
                end
            end
            else begin
                wen <= 1'b1;
            end
        end
    end

endmodule

module ll_display(
    input logic clk, 
    input logic rst, 
    input logic land,
    input logic crash, 
    input logic [3:0] disp_ctrl, 
    input logic [15:0] alt,
    input logic [15:0] vel, 
    input logic [15:0] fuel, 
    input logic [15:0] thrust, 
    output logic [7:0] ss7, ss6, ss5, ss3, ss2, ss1, ss0, 
    output logic red,
    output logic green
    );
    // Intermediate Registers
    logic [4:0] hold;
    logic [3:0] display_ctrl;
    logic [15:0] alt_new;
    logic [3:0] mode;
    logic [3:0] new_mode;
    logic [15:0] neg;
    logic [15:0] num;
    logic [23:0] state;
    logic [6:0] display;

    // Arithmetic
    bcdaddsub4 convert(.a(0), .b(vel), .op(1'b1), .s(neg));

    // FSM
    always_comb begin
      {ss7, ss6, ss5} = 24'b011101110011100001111000;
      num = alt;
      case (mode)
        4'b0001: begin
          {ss7, ss6, ss5} = 24'b011110000111011001010000;
          num = thrust;
        end
        4'b0010: begin
          {ss7, ss6, ss5} = 24'b011011110111011101101101;
          num = fuel;
        end
        4'b0100: begin
          {ss7, ss6, ss5} = 24'b001111100111100100111000;
          num = (vel[15] == 1) ? neg : vel;
        end
        4'b1000: begin
          {ss7, ss6, ss5} = 24'b011101110011100001111000;
          num = alt;
        end
        default: begin
          {ss7, ss6, ss5} = 24'b011110000111011001010000;
          num = thrust;
        end
      endcase
    end

    // Flip Flop
    always_ff @(posedge clk, posedge rst) begin : FF
      if (rst) begin
        mode <= 4'b1000;
      end
      else begin
        mode <= disp_ctrl;
      end
    end

    // Display Instantiations + LED Assignment
    ssdec display1(.in(num[3:0]), .enable(1'b1), .out(ss0[6:0]));
    ssdec display2(.in(num[7:4]), .enable(|num[15:4]), .out(ss1[6:0]));
    ssdec display3(.in(num[11:8]), .enable(|num[15:8]), .out(ss2[6:0]));
    assign ss3 = ({ss7, ss6, ss5} == 24'b001111100111100100111000 && vel[15] == 1) ? 8'b1000000 : {1'b0, display};
    ssdec display4(.in(num[15:12]), .enable(|num[15:12]), .out(display[6:0]));
    assign red = rst ? 1'b0 : crash;
    assign green = rst ? 1'b0 : land;

endmodule

module lunarlander #(
    parameter FUEL=16'h800,
    parameter ALTITUDE=16'h4500,
    parameter VELOCITY=16'h0,
    parameter THRUST=16'h5,
    parameter GRAVITY=16'h5
    )(
    input logic hz100, reset,
    input logic [19:0] in,
    output logic [7:0] ss7, ss6, ss5,
    output logic [7:0] ss3, ss2, ss1, ss0,
    output logic red, green
    );
    // Intermediate Registers
    logic psc_clock;
    logic clock_sync;
    logic[4:0] ll_keyout;
    logic[15:0] altitude;
    logic[15:0] velocity;
    logic[15:0] llfuel;
    logic[15:0] llthrust;
    logic[15:0] llthrust_2;
    logic[15:0] altitude_2;
    logic[15:0] velocity_2;
    logic[15:0] llfuel_2;
    logic ll_wen;
    logic ll_clock;
    logic ll_land;
    logic ll_crash;

    // Module Instaantiations
    clock_psc clock(.clk(hz100), .lim(8'd24), .rst(reset), .hzX(psc_clock));
    keysync ll_sync(.rst(reset), .keyin(in[19:0]), .clk(hz100), .keyclk(clock_sync), .keyout(ll_keyout));

    // Flip Flop
    always_ff @(posedge clock_sync, posedge reset) begin
        if(reset) begin
            llthrust_2 = 16'h5;
        end else if((~ll_keyout[4])) begin
            llthrust_2 = {12'b0, ll_keyout[3:0]};
        end
    end

    // Module Instantiations
    ll_alu arithmetic(.alt(altitude), .fuel(llfuel), .vel(velocity), .thrust(llthrust), .alt_n(altitude_2), .vel_n(velocity_2), .fuel_n(llfuel_2));
    ll_memory memory(.clk(psc_clock), .rst(reset), .wen(ll_wen), 
                     .alt_n(altitude_2), .vel_n(velocity_2), .fuel_n(llfuel_2), .thrust_n(llthrust_2),
                     .alt(altitude), .fuel(llfuel), .vel(velocity), .thrust(llthrust));
    ll_display display(.clk(clock_sync), .rst(reset), .alt(altitude), .fuel(llfuel), .vel(velocity), .thrust(llthrust),
                      .disp_ctrl({ll_keyout == 5'd19, ll_keyout == 5'd18, ll_keyout == 5'd17, ll_keyout == 5'd16}), .land(ll_land), .crash(ll_crash),
                      .ss7(ss7), .ss6(ss6), .ss5(ss5), .ss3(ss3),
                      .ss2(ss2), .ss1(ss1), .ss0(ss0), .red(red), .green(green));

    ll_control control(.alt(altitude), .vel(velocity), .clk(psc_clock), .rst(reset), .crash(ll_crash), .land(ll_land), .wen(ll_wen));

endmodule

module fa (
    input a, 
    input b, 
    input ci, 
    output s, 
    output co
    );
    assign s = a ^ b ^ ci;
    assign co = (a & b) | (b & ci) | (a & ci);

endmodule

module fa4 (
    input [3:0] a, 
    input [3:0] b, 
    input ci, 
    output [3:0] s, 
    output co
    );
    // Intermediate Registers
    logic [3:0] co_intermediate;
    // 1 Bit Full Adders
    fa fa0 (.a(a[0]), .b(b[0]), .ci(ci), .s(s[0]), .co(co_intermediate[0]));
    fa fa1 (.a(a[1]), .b(b[1]), .ci(co_intermediate[0]), .s(s[1]), .co(co_intermediate[1]));
    fa fa2 (.a(a[2]), .b(b[2]), .ci(co_intermediate[1]), .s(s[2]), .co(co_intermediate[2]));
    fa fa3 (.a(a[3]), .b(b[3]), .ci(co_intermediate[2]), .s(s[3]), .co(co_intermediate[3]));
    // Assign Carry Out
    assign co = co_intermediate[3];

endmodule

module bcdadd1 (
    input [3:0] a, 
    input [3:0] b, 
    input ci, 
    output [3:0] s, 
    output co
    );
    // Intermediate Registers
    logic fa4_co;
    logic [3:0]fa4_sum;
    logic co_int;

    // 4 Bit Adder Instantiations
    fa4 fa4_instance (.a(a), .b(b), .ci(ci), .s(fa4_sum), .co(fa4_co));
    assign co_int = (fa4_sum[3] & fa4_sum[2]) | (fa4_sum[3] & fa4_sum[1]) | (fa4_co);
    fa4 fa4_instance2 (.a({1'b0, co_int, co_int, 1'b0}), .b(fa4_sum), .ci(1'b0), .s(s), .co());
    assign co = co_int;

endmodule

module bcdadd4 (
    input [15:0] a, 
    input [15:0] b, 
    input ci, 
    output [15:0] s, 
    output co
    );
    // Intermediate Registers
    logic [3:0] a_digit, b_digit;
    logic [3:0] carry_out;
    logic carry_in;
    assign carry_in = ci;

    // BCD Adders
    bcdadd1 bcd_adder0 (.a(a[3:0]), .b(b[3:0]), .ci(carry_in), .s(s[3:0]), .co(carry_out[0]));
    bcdadd1 bcd_adder1 (.a(a[7:4]), .b(b[7:4]), .ci(carry_out[0]), .s(s[7:4]), .co(carry_out[1]));
    bcdadd1 bcd_adder2 (.a(a[11:8]), .b(b[11:8]), .ci(carry_out[1]), .s(s[11:8]), .co(carry_out[2]));
    bcdadd1 bcd_adder3 (.a(a[15:12]), .b(b[15:12]), .ci(carry_out[2]), .s(s[15:12]), .co(carry_out[3]));
    
    assign co = carry_out[3];

endmodule

module bcd9comp1 (
    input [3:0] in, 
    output reg [3:0] out
    );
    // Case Assignments
    always @(*) begin
        case (in)
            4'b0000: out = 4'b1001;
            4'b0001: out = 4'b1000;
            4'b0010: out = 4'b0111;
            4'b0011: out = 4'b0110;
            4'b0100: out = 4'b0101;
            4'b0101: out = 4'b0100;
            4'b0110: out = 4'b0011;
            4'b0111: out = 4'b0010;
            4'b1000: out = 4'b0001;
            4'b1001: out = 4'b0000;
            default: out = 4'b1001;
        endcase
    end

endmodule

module bcdaddsub4 (
    input [15:0] a, 
    input [15:0] b, 
    input op, 
    output [15:0] s
    );
    // Intermediate Registers
    logic [15:0] c;

    // Adders
    bcdadd4 add(.a(a), .b(op ? c : b), .ci(op), .s(s), .co());
    bcd9comp1 tc_a0 (.in(b[3:0]), .out(c[3:0]));
    bcd9comp1 tc_a1 (.in(b[7:4]), .out(c[7:4]));
    bcd9comp1 tc_a2 (.in(b[11:8]), .out(c[11:8]));
    bcd9comp1 tc_a3 (.in(b[15:12]), .out(c[15:12]));
    
endmodule

module clock_psc (
    input logic clk, 
    input logic rst, 
    input logic [7:0] lim, 
    output logic hzX
    );
    // Intermediate Registers
    logic [7:0] counter;

    // Flip Flop
    always_ff @(posedge clk or posedge rst) begin : FF
        if (rst) begin
            counter <= 8'b0;
            hzX <= 1'b0;
        end else if(lim == 8'b0) begin
            counter <= 8'b0;
            hzX <= 1'b0;
        end else begin
            if (counter == lim) begin
                counter <= 8'b0;
                hzX <= ~hzX;
            end else begin
                counter <= counter + 1;
            end
        end
    end

endmodule

module  keysync(
    input logic clk,
    input logic rst, 
    input logic [19:0] keyin, 
    output logic [4:0]keyout, 
    output logic keyclk
    );
    // Intermediate Registers/Logic
    logic keyclock;
    assign keyclock = |keyin[19:0];
    logic [1:0] delay;

    // Flip Flop
    always_ff @ (posedge clk) begin : FF
    if (rst == 1'b1)
      delay[1] <= 0;
    else
      delay <= (delay << 1) | {1'b0, keyclock};
    end

    assign keyclk = delay[1];

    assign keyout[0] = keyin[1] | keyin[3] | keyin[5] | keyin[7] | keyin[9] | keyin[11] | keyin[13] | keyin[15] | keyin[17] | keyin[19];
    assign keyout[1] = keyin[2] | keyin[3] | keyin[6] | keyin[7] | keyin[10] | keyin[11] | keyin[14] | keyin[15] | keyin[18] | keyin[19];
    assign keyout[2] = (| keyin[7:4]) | (| keyin[15:12]);
    assign keyout[3] = | keyin[15:8];
    assign keyout[4] = | keyin[19:16];

endmodule

module ssdec (
    input logic [3:0] in, 
    input logic enable, 
    output logic [6:0] out
    );
    // Intermediate Array
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

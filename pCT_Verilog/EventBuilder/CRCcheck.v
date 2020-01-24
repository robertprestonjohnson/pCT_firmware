//Program to calculate a 6-bit CRC to check the data stream by comparing with
//the CRC calculated on the tracker or energy detector front-end board
//R. Johnson
//Modified August 12, 2013 to remove some unnecessary stuff
module CRCcheck(CRC,Din,Stop,Start,Reset,Clock,Address);
input [3:0] Address;
input Clock;
input Reset;
input Stop;       		//Must go high one clock cycle before the last bit of data
input Start;
input Din;        		//Data stream in
output [5:0] CRC;       //6-bit CRC output

parameter [5:0] Key= 6'b100101;   //7-bit key by which the data packet is divided is {1'b1,Key}.  The 6-bit CRC is the remainder.
reg [6:0] Rmdr;

//Definition of states for the state machine:
parameter [4:0] Idle= 5'b00001;
parameter [4:0] Wait= 5'b00010;    //Shift data into Rmdr
parameter [4:0] DXor= 5'b00100;    //Xor Key with Rmdr; result goes back into Rmdr[6:1], and next data bit goes into Rmdr[0]
parameter [4:0] Paus= 5'b01000;    //Pause one clock cycle to allow the last data bit to go out
parameter [4:0] ShOt= 5'b10000;    
reg [4:0] State, NextState;

//reg [2:0] Cnt;
//reg [23:0] DinOld;
reg Stop2;        //Set if there was a Stop on previous clock cycle

//initial begin
//	$display("Key=%b",{1'b1,Key});
//	$display("Time    State  Din Dout   Rmdr Stop Stop2 Reset Done Cnt");
//end

//always @ (posedge Clock) begin
//	$display("%g\t %b    %b    %b  %b %b    %b    %b    %b     %d",$time,State,Din,Dout,Rmdr,Stop,Stop2,Reset,Done,Cnt);
//end

assign CRC = Rmdr[6:1];

always @ (State or Stop or Din or Rmdr or Stop2 or Start)
begin 
	case (State)
		Idle:	begin
					if (Start) NextState = Wait;
					else NextState = Idle;
				end
		Wait:	begin
					if (Rmdr[5] == 1'b1) NextState = DXor;
					else if (Stop) NextState = Paus;
					else NextState = Wait;
				end
		DXor:	begin
					if (Stop) NextState = Paus;
					else if (Stop2) begin NextState = ShOt; end
					else if (Key[5]^Rmdr[5]) NextState = DXor;
					else NextState = Wait;
				end
		Paus:   begin
					NextState = ShOt;
		        end
		ShOt:	begin
					NextState = Idle;
				end
		default: 	begin
						NextState = Idle;
					end
	endcase
end

always @ (posedge Clock)
begin
	if (Reset) begin
		State <= Idle;
		Rmdr <= 0;
	end else begin
//		if (Stop) $display("%g\t CRCcheck %h state %b, Stop signal received, Din=%b.",$time,Address,State,Din);
		State <= NextState;
		Stop2 <= Stop;
//		DinOld <= {DinOld[22:0],Din};
//		if (State != Idle && Address == 4'd7) $display("%g\t CRCcheck %h state %b:  Din=%b, DinOld=%b,  Rmdr=%b, Stop=%b, Stop2=%b",$time,Address,State,Din,DinOld,Rmdr,Stop,Stop2);
		case (State)
			Idle:	begin
						if (NextState == Wait) Rmdr <= {6'b000000,Din};
					end
			Wait:	begin
//						Cnt <= 0;
						Rmdr <= {Rmdr[5:0],Din};
					end
			DXor:	begin
						Rmdr[6:1] <= Rmdr[5:0]^Key;
						Rmdr[0] <= Din;
					end
			Paus:   begin
						if (Rmdr[6]==1'b1) begin
							Rmdr[6:1] <= Rmdr[5:0]^Key;
							Rmdr[0] <= 1'b0;
						end else Rmdr <= {Rmdr[5:0],1'b0};
					end
		endcase
	end
end

endmodule

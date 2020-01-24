//Program to calculate a 6-bit CRC on the fly and append it to the end of a the serial data stream
//R. Johnson, 2013
//August 14, 2013:  added Address to arg list, for debug work
module CRC6(Dout,Done,Din,Stop,Reset,Clock,Address);
input [3:0] Address;
input Clock;
input Reset;
input Stop;       //Must go high one clock cycle before the last bit of data
input Din;        //Data stream in
output Dout;      //Data stream Din delayed by one clock, with 6-bit CRC appended to the end with no gap
output Done;      //Signals when done & ready for the next packet

reg Dout, Done;

parameter [5:0] Key= 6'b100101;   //7-bit key by which the data packet is divided is {1'b1,Key}.  The 6-bit CRC is the remainder.
reg [6:0] Rmdr;

//Definition of states for the state machine:
parameter [3:0] Wait= 4'b0001;    //Shift data into Rmdr
parameter [3:0] DXor= 4'b0010;    //Xor Key with Rmdr; result goes back into Rmdr[6:1], and next data bit goes into Rmdr[0]
parameter [3:0] Paus= 4'b0100;    //Pause one clock cycle to allow the last data bit to go out
parameter [3:0] ShOt= 4'b1000;    //Shift the CRC out
reg [3:0] State, NextState;

reg [2:0] Cnt;
reg Stop2;        //Set if there was a Stop on previous clock cycle
reg DDly;         //Data delayed by 1 clock cycle
//reg [23:0] DinOld;

//initial begin
//	$display("Key=%b",{1'b1,Key});
//	$display("Time    State  Din Dout   Rmdr Stop Stop2 Reset Done Cnt");
//end

//always @ (posedge Clock) begin
//	$display("%g\t %b    %b    %b  %b %b    %b    %b    %b     %d",$time,State,Din,Dout,Rmdr,Stop,Stop2,Reset,Done,Cnt);
//end

always @ (State or Stop or Din or Rmdr or Cnt or DDly or Stop2)
begin 
	case (State)
		Wait:	begin
					if (Rmdr[5] == 1'b1) NextState = DXor;
					else if (Stop) NextState = Paus;
					else NextState = Wait;
					Dout = DDly;
				end
		DXor:	begin
					if (Stop) NextState = Paus;
					else if (Stop2) begin NextState = ShOt; end
					else if (Key[5]^Rmdr[5]) NextState = DXor;
					else NextState = Wait;
					Dout = DDly;
				end
		Paus:   begin
					NextState = ShOt;
					Dout = DDly;
		        end
		ShOt:	begin
					Dout = Rmdr[6];
					if (Cnt == 5) NextState = Wait;
					else NextState = ShOt;
				end
		default: 	begin
						NextState = Wait;
						Dout = DDly;
					end
	endcase
end

always @ (posedge Clock)
begin
	if (Reset) begin
		State <= Wait;
		Rmdr <= 0;
	end else begin
		State <= NextState;
//		if (Stop) $display("%g\t CRC6 %h state %b, Stop signal received, Din=%b.",$time,Address,State,Din);
		Stop2 <= Stop;
//		DinOld <= {DinOld[22:0],Din};
		DDly <= Din;
//		if ((State != Wait || NextState != Wait) && Address == 4'd7) $display("%g\t CRC6 %d state %b: Din=%b, DinOld=%b DDly=%b, Rmdr=%b, Done=%b, Dout=%b, Cnt=%d, Stop=%b",$time,Address,State,Din,DinOld,DDly,Rmdr,Done,Dout,Cnt,Stop); 
		case (State)
			Wait:	begin
						Cnt <= 0;
						Done <= 0;
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
			ShOt:	begin
						Cnt <= Cnt + 1;
						Rmdr <= {Rmdr[5:0],1'b0};
						if (Cnt == 5) Done <= 1'b1;
					end
		endcase
	end
end

endmodule

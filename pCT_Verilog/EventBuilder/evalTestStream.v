module evalTestStream(Ntests,badTest,Nerr,Done,Stream,Start,Clock,Reset);
// Check the test stream coming from a front-end board for any bit errors.
// R. Johnson  April 4, 2014

input Clock;
input Reset;
input Start;		//Goes high for one clock cycle to start data streaming
input Stream;		//Serial input stream
output Done;		//1-clock signal when complete
output [15:0] Nerr;	//Number of word errors detected
output [23:0] Ntests; //Number of word tests executed
output [23:0] badTest; //Number of the last failed test

reg Done;
reg [15:0] Nerr;
reg [23:0] Ntests, badTest;
reg [23:0] Word, Expected;
reg [11:0] CntSafe;

parameter [4:0] Wait=5'b00001;	//Wait for a start signal
parameter [4:0] Look=5'b00010;  //Look for the start bit
parameter [4:0] Shft=5'b00100;	//Shift in the 32 bits
parameter [4:0] Coun=5'b01000;	//Check the 32-bit word
parameter [4:0] Quit=5'b10000;	//End of the test

reg [4:0] State, NextState;
reg [4:0] CntBits;

always @ (State or Start or CntBits or Expected or Stream or CntSafe) begin
	case (State)
		Wait:	begin
					if (Start) NextState = Look;
					else NextState = Wait;
				end
		Look:	begin
					if (CntSafe == 12'hfff) NextState = Wait;
					else if (Stream) NextState = Shft;
					else NextState = Look;
				end
		Shft:	begin
					if (CntBits == 22) NextState = Coun;
					else NextState = Shft;
				end
		Coun:	begin	
					if (Expected == 1) begin
						NextState = Quit;
					end else begin
						NextState = Shft;
					end
				end
		Quit:	begin	
					NextState = Wait;
				end
		default:	begin
						NextState = Wait;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
	end else begin
		State <= NextState;
		if (State != Wait) $display("%g\t evalTestStream: State=%b, Stream=%b, Word=%b, Expected=%d, Nerr=%d, CntBits=%d, Done=%b",$time,State,Stream,Word,Expected,Nerr,CntBits,Done);
		case (State)
			Wait:	begin
						CntBits <= 0;
						Done <= 1'b0;
						CntSafe <= 0;
					end
			Look:	begin
						CntSafe <= CntSafe + 1;
						if (CntSafe == 12'hfff) Done <= 1'b1;   //Abort if no test data show up.
						Expected <= 24'hffffff;
						Nerr <= 0;
						Ntests <= 0;
						badTest <= 0;
						Word[0] <= Stream;
					end
			Shft:	begin
						CntBits <= CntBits + 1;
						Word <= {Word[22:0],Stream};
					end
			Coun:	begin	
						Expected <= Expected - 1;
						if (Word != Expected && Nerr<16'hffff) begin
							Nerr <= Nerr + 1;
							badTest <= Expected;
						end
						Ntests <= Ntests + 1;
						CntBits <= 0;		
						Word <= {Word[22:0],Stream};
					end
			Quit:	begin
						Done <= 1'b1;
					end
		endcase
	end
end

endmodule

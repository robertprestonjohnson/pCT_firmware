module genTestStream(Done,Stream,Start,Clock,Reset,Address);
//Produce a test stream to send to the event builder to test the data transmission lines.
//R. Johnson    April 4, 2014

input Clock;
input Reset;
input Start;		//Goes high for one clock cycle to start data streaming
input [3:0] Address;
output Stream;		//Serial output stream
output Done;		//1-clock signal when complete

reg Done, Stream;
reg [23:0] Word;

parameter [3:0] Wait=4'b0001;	//Wait for a start signal
parameter [3:0] Shft=4'b0010;	//Shift out the bits
parameter [3:0] Coun=4'b0100;	//Decrement the test word
parameter [3:0] Quit=4'b1000;	//End of the test

reg [3:0] State, NextState;
reg [4:0] CntBits;

always @ (State or Start or CntBits or Word) begin
	case (State)
		Wait:	begin
					if (Start) NextState = Shft;
					else NextState = Wait;
					Stream = 1'b0;
				end
		Shft:	begin
					if (CntBits == 22) NextState = Coun;
					else NextState = Shft;
					Stream = Word[23];
				end
		Coun:	begin	
					if ({Word[22:0],Word[23]} == 1) begin
						NextState = Quit;
					end else begin
						NextState = Shft;
					end
					Stream = Word[23];
				end
		Quit:	begin	
					NextState = Wait;
					Stream = 1'b0;
				end
		default:	begin
						NextState = Wait;
						Stream = 1'b0;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		Word <= 24'hffffff;
	end else begin
		State <= NextState;
//		if (State != Wait) $display("%g\t getTestStream %d: State=%b, CntBits=%d, Word=%b, Stream=%b, Done=%b",$time,Address,State,CntBits,Word,Stream,Done);
		case (State)
			Wait:	begin
						CntBits <= 0;
						Done <= 1'b0;
					end
			Shft:	begin
						CntBits <= CntBits + 1;
						Word <= {Word[22:0],Word[23]};
					end
			Coun:	begin	
						Word <= {Word[22:0],Word[23]} - 1;
						CntBits <= 0;						
					end
			Quit:	begin
						Word <= 24'hffffff;
						Done <= 1'b1;
					end
		endcase
	end
end

endmodule

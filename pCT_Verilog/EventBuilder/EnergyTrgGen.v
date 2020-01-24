module EnergyTrgGen(
    output Trigger,
    input Clock,
    input Reset
    );

reg Trigger;
reg [18:0] Cnt;

always @ (posedge Clock) begin
	if (Reset) begin
		Cnt <= 0;
		Trigger <= 0;
	end else begin
		Cnt <= Cnt + 1;
		if (Cnt == 19'b1111111111111111111) Trigger <= 1'b1;
		else Trigger <= 1'b0;
	end
end

endmodule

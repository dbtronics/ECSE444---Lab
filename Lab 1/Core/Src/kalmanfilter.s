
.syntax unified

.global kalmanfilter


kalmanfilter:
	PUSH {R4, R5}
	PUSH {LR} //preserve the origional link register state on the stack
	MOV R6,0x0FF
	LSL R6, R6, #23 //set test bit

	LDR R7, [R1] //validate measurement
	BL validate  //replace the link register state in r14 with pc value
	VLDR.f32 S0, [R1] //accept measurement value

	LDR R7, [R0] //validate self.q
	BL validate
	VLDR.f32 S1,[R0]

	LDR R7, [R0, #4] //validate self.r
	BL validate
	VLDR.f32 S2, [R0, #4]

	VLDR.f32 S3, [R0, #8] //self.x
	VLDR.f32 S4, [R0, #12] //self.p
	VLDR.f32 S5, [R0, #16] //self.k

	POP {LR} //restore the origional link register state
	B update


validate:
	AND R8, R7,R6
	LSL R8, R8, #1 //remove sign bit
	LSR R8, R8, #24 //remove mantisa
	CMP R8, #1 //min number in exponent
	BLT error
	CMP R8, #255 //max number in exponent
	BGE error
	BX LR

error:
	POP {LR}
	B done


update:
	VADD.f32 S4, S4, S1 //self.p = self.p + self.q
	VADD.f32 S6, S4, S2 //self.p + self.r
	VDIV.f32 S5, S4, S6 //slef.k = self.p /(self.p + self.r)
	VSUB.f32 S7, S0, S3 //measurement - self.x
	VMUL.f32 S8, S5, S7 //self.k * (measurement - self.x)
	VADD.f32 S3, S3, S8 //self.x = self.x + self.k * (measurement - self.x)
	VMUL.f32 S9, S4, S5 //self.p * self.k
	VSUB.f32 S4, S4, S9 //self.p = self.p - (self.p * self.k)

	VSTR.f32 S1, [R0] //store state
	VSTR.f32 S2, [R0, #4]
	VSTR.f32 S3, [R0, #8]
	VSTR.f32 S4, [R0, #12]
	VSTR.f32 S5, [R0, #16]

done:
	POP {R4, R5}
	BX LR

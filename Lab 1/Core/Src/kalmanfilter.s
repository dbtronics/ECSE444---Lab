
.syntax unified

.global kalmanfilter

//section marks a new section in assembly.
//.text identifies it as source code
//rodata marks it as read only, setting it to go into flash not SRAM
.section .text.rodata

kalmanfilter:
	PUSH {R4 - R9} //save register states
	PUSH {LR} //preserve the origional link register state on the stack

	MOV R6,0x0FF
	LSL R6, R6, #23 //set test bit

	VLDR.f32 S0, [R1] //accept measurement value
	VLDR.f32 S1,[R0]
	VLDR.f32 S2, [R0, #4]
	VLDR.f32 S3, [R0, #8]
	VLDR.f32 S4, [R0, #12]
	VLDR.f32 S5, [R0, #16]
	POP {LR} //restore the origional link register state
	B update


validate:
	AND R8, R7,R6 //perform AND operation on binary value in R7 and binary testbit in R6
	LSL R8, R8, #1 //remove sign bit
	LSR R8, R8, #24 //remove mantisa
	CMP R8, #1 //min number in exponent
	BLT error
	CMP R8, #254 //max number in exponent
	BGT error
	BX LR

error:
	MOV R9, 0 //set if invalid
	STR R9, [R2]
	POP {LR}
	B done


update:
	PUSH {LR} //preserve origional LR state on stack
	VADD.f32 S4, S4, S1 //self.p = self.p + self.q

	VADD.f32 S6, S4, S2 //self.p + self.r

	VMOV.f32 R7, S6
	BL validate // validate (self.p + self.r) !=0

	VDIV.f32 S5, S4, S6 //self.k = self.p /(self.p + self.r)
	VSUB.f32 S7, S0, S3 //measurement - self.x

	VMOV.f32 R7, S7
	BL validate //validate (measurement -self.x) !=INF or overflow

	VMUL.f32 S8, S5, S7 //self.k * (measurement - self.x)
	VMOV.f32 R7, S8
	BL validate // validate self.k * (measurement - self.x) !=INF or overflow

	VADD.f32 S3, S3, S8	//self.x = self.x + self.k * (measurement - self.x)

	VMUL.f32 S9, S4, S5 //self.p * self.k

	VSUB.f32 S4, S4, S9 //self.p = self.p - (self.p * self.k)


	VSTR.f32 S1, [R0] //store states, self.q
	VSTR.f32 S2, [R0, #4] //self.r
	VSTR.f32 S3, [R0, #8] //self.x
	VSTR.f32 S4, [R0, #12] //self.p
	VSTR.f32 S5, [R0, #16] //self.k

	MOV R9, 1 //set if everything is valid
	STR R9, [R2]
	POP {LR} //restore origional LR state on stack

done:
	POP {R4 - R9}
	BX LR


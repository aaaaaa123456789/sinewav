%imacro assert 1-2 "assertion failed"
	%ifn %1
		%error %2
	%endif
%endmacro

%idefine double(value) __?float64?__(value)

%assign BUFFER_PAGES 32

struc globals
	; no local labels - these are singletons! Access via [rbp + xxxx]
	OutputBufferPointer: resq 1 ; equals end of wave offsets
	WaveOffsetPointer:   resq 1 ; for each wave, 16 offsets: (+16, +1, +2, ..., +15) * frequency % output frequency
	WaveformCount:       resd 1
	OutputFrequency:     resd 1
	OutputLength:        resq 1
	OutputVolume:        resq 1
	Temp:                resq 1 ; buffer for any operation that must go through memory
	FPControlWord:       resd 1
	FPCWTruncating:      resd 1
	ArgumentFlags:       resw 1 ; bit 15: length in samples; options set: 0: frequency, 1: length, 2: mode, 3: volume
	OutputMode:          resb 1 ; 0: bytes, 1: words, 2: floats
		resb 5
	SampleBuffer:        resq 4 ; samples for the wave being generated
	Waveforms:
endstruc

struc waveform
	.offset:    resq 1 ; current value for the wave
	.amplitude: resq 1
	.frequency: resq 1
	.phase:     resq 1
endstruc

%imacro withend 1+
	%defstr %%label %00
	%substr %%sb %%label 1
	%deftok %%firstchar %%sb
	%substr %%sb %%label 2,-1
	%deftok %%remainder %%sb
	%ifidn %[%%firstchar],.
		%define %%endlabel .%[%%remainder]_end
	%else
		%define %%endlabel .end
	%endif
%00:
	%1
%[%%endlabel]:
%endmacro

; Linux x64 syscall IDs
%assign write        1
%assign mmap         9
%assign fcntl       72
%assign exit_group 231

; errno values
%assign EINTR 4
%assign EBADF 9

; kernel API constants and flags
%assign F_GETFD          1
%assign MAP_PRIVATE      2
%assign MAP_ANONYMOUS 0x20
%assign PROT_READ        1
%assign PROT_WRITE       2

; comparison constants (for vcmppd, etc.) - default ordered (unless negated) and non-signalling
%assign CMP_NOT_EQ    4
%assign CMP_LT     0x11
%assign CMP_GE     0x1d
%assign CMP_GT     0x1e

	section .text align=1


	global _start:function
_start:
	mov edi, 2
	mov esi, F_GETFD
	mov eax, fcntl
	syscall
	cmp rax, -EBADF
	jz ExitFailure ; stderr is closed: just fail
	mov esi, F_GETFD
	assert F_GETFD == 1
	mov edi, esi
	mov eax, fcntl
	syscall
	cmp rax, -EBADF
	jz .nooutput
	pop rsi ; argument count + 1
	dec rsi ; assume worst case (one waveform per argument)
	assert waveform_size == 32
	shl rsi, 5
	pop r15 ; store the program name somewhere
	lea r12, [rsi * 4] ; for the wave offset buffer, 16 doubles per wave
	lea rsi, [rsi + r12 + globals_size + 0xfff + BUFFER_PAGES * 0x1000]
	and rsi, -0x1000
	lea r14, [rsi - BUFFER_PAGES * 0x1000]
	xor edi, edi
	mov edx, PROT_READ | PROT_WRITE
	mov r10, MAP_PRIVATE | MAP_ANONYMOUS
	mov r8, -1
	xor r9d, r9d
	mov eax, mmap
	syscall
	cmp rax, -0x1000
	jnc .allocfail
	mov rbp, rax
	lea rax, [rbp + r14]
	mov [rbp + OutputBufferPointer], rax
	sub rax, r12
	mov [rbp + WaveOffsetPointer], rax
	; initialize some options (everything else is zero-initialized by mmap)
	mov dword[rbp + OutputFrequency], 44100
	mov rax, double(5.)
	mov [rbp + OutputLength], rax
	mov byte[rbp + OutputMode], 1
	lea r12, [rbp + Waveforms]
.argloop:
	pop rsi
	test rsi, rsi
	jz GenerateOutput
	mov rdi, [rsp] ; next argument, for options that need it
	call ProcessArgument
	jnc .argloop
	add rsp, 8
	jmp .argloop

.allocfail:
	lea rsi, [rel Messages.allocation_failure]
	mov edx, Messages.allocation_failure_end - Messages.allocation_failure
	jmp ErrorExit

.nooutput:
	lea rsi, [rel Messages.no_stdout]
	mov edx, Messages.no_stdout_end - Messages.no_stdout
ErrorExit:
	mov edi, 2
	mov eax, write
	syscall
ExitFailure:
	mov edi, 1
	mov eax, exit_group
	syscall
	ud2

GenerateOutput:
	cmp dword[rbp + WaveformCount], 0
	jz ProcessArgument.usage
	fld1
	fchs
	fild dword[rbp + OutputFrequency]
	fst qword[rbp + Temp]
	fscale
	vbroadcastsd ymm10, [rbp + Temp]
	fstp st1
	fst qword[rbp + Temp]
	vbroadcastsd ymm15, [rbp + Temp]
	fldpi
	fdivrp st1
	fstp qword[rbp + Temp]
	vbroadcastsd ymm11, [rbp + Temp]
	; ymm10 = frequency, ymm11 = 2pi / frequency, ymm15 = frequency / 2
	test byte[rbp + ArgumentFlags + 1], 0x80
	jnz .gotsamples
	vmulsd xmm0, xmm10, [rbp + OutputLength]
	vcvtsd2si rax, xmm0
	lea rsi, [rel Messages.bad_frequency_length]
	mov edx, Messages.bad_frequency_length_end - Messages.bad_frequency_length
	cmp rax, 1000000000
	jnc ErrorExit
	test eax, eax
	jz ErrorExit
	mov [rbp + OutputLength], eax
.gotsamples:
	; convert the volume from negative dB to a factor
	fld qword[rbp + OutputVolume]
	fldl2t
	fmulp st1
	mov word[rbp + Temp], -20
	fild word[rbp + Temp]
	fdivp st1
	fld st0
	frndint
	fxch st1
	fsub st0, st1
	f2xm1
	fld1
	faddp st1
	fscale
	fstp st1
	fstp qword[rbp + OutputVolume]

	; generate the .wav header (fixed size, up to the header of the data chunk)
	mov eax, [rbp + OutputLength]
	mov ebx, eax
	movzx ecx, byte[rbp + OutputMode]
	shl ebx, cl
	mov edi, 1
	and edi, ebx
	add ebx, edi
	mov rdi, [rbp + OutputBufferPointer]
	mov dword[rdi], 0x46464952 ; "RIFF" (backwards)
	lea edx, [ebx + 50]
	mov [rdi + 4], edx
	mov rdx, 0x20746d6645564157 ; "WAVEfmt " (backwards)
	mov [rdi + 8], rdx
	mov dword[rdi + 16], 18
	xor edx, edx
	cmp cl, 2
	cmovz edx, ecx
	add edx, 0x10001 ; format = 1 (PCM) or 3 (float); channels = 1
	mov [rdi + 20], edx
	mov edx, [rbp + OutputFrequency]
	mov [rdi + 24], edx
	shl edx, cl
	mov [rdi + 28], edx
	mov [rdi + 32], cx
	mov edx, 8
	shl edx, cl
	mov [rdi + 34], dx
	mov word[rdi + 36], 0
	mov rdx, 0x474636166 ; "fact" (backwards), 4
	mov [rdi + 38], rdx
	mov [rdi + 46], eax
	mov dword[rdi + 50], 0x61746164 ; "data" (backwards)
	mov [rdi + 54], ebx
	mov ebx, 58
	call WriteOutput

	; prepare data to be kept in registers
	; r12 = remaining samples, r13 = generator function, r14 = buffer size (generated samples)
	; already prepared (broadcast): ymm10 = frequency, ymm11 = 2pi / frequency, ymm15 = frequency / 2
	; broadcasts: ymm12 = 1, ymm13 = -1, ymm14 = volume
	mov r12d, [rbp + OutputLength]
	movzx ecx, byte[rbp + OutputMode]
	lea rsi, [rel SampleGenerators]
	mov r13, [rsi + rcx * 8]
	mov r14d, BUFFER_PAGES * 0x1000
	shr r14d, cl
	mov rax, double(1.)
	mov [rbp + Temp], rax
	vbroadcastsd ymm12, [rbp + Temp]
	vxorpd xmm0, xmm0, xmm0
	vsubpd ymm13, ymm0, ymm12
	vbroadcastsd ymm14, [rbp + OutputVolume]
	; store current FP control word, and create an equivalent with the rounding mode set to truncating
	vstmxcsr [rbp + FPControlWord]
	mov eax, [rbp + FPControlWord]
	bts eax, 13
	btr eax, 14
	mov [rbp + FPCWTruncating], eax

	lea rsi, [rbp + Waveforms]
	mov rdi, [rbp + WaveOffsetPointer]
	mov ecx, [rbp + WaveformCount]
.wavesetup:
	vmovq xmm0, [rsi + waveform.phase]
	vroundsd xmm1, xmm0, xmm0, 1
	vsubsd xmm0, xmm0, xmm1
	vmulsd xmm0, xmm0, xmm10
	vmovq [rsi + waveform.offset], xmm0
	vmovq xmm0, [rsi + waveform.frequency] ; sets other values in xmm0 to zero
	vaddsd xmm1, xmm0, xmm0
	vpermilpd xmm0, xmm0, 1
	vpermilpd xmm1, xmm1, 0
	vsubpd xmm7, xmm1, xmm10
	vblendvpd xmm1, xmm7, xmm1, xmm7
	vaddpd xmm6, xmm0, xmm1
	vinsertf128 ymm1, ymm1, xmm1, 1
	vsubpd xmm7, xmm6, xmm10
	vblendvpd xmm6, xmm7, xmm6, xmm7
	vinsertf128 ymm0, ymm0, xmm6, 1
	vaddpd ymm1, ymm1, ymm1
	vsubpd ymm7, ymm1, ymm10
	vblendvpd ymm1, ymm7, ymm1, ymm7
	vaddpd ymm2, ymm1, ymm1
	vsubpd ymm6, ymm2, ymm10
	vaddpd ymm1, ymm1, ymm0
	vsubpd ymm7, ymm1, ymm10
	vblendvpd ymm2, ymm6, ymm2, ymm6
	vblendvpd ymm1, ymm7, ymm1, ymm7
	vaddpd ymm3, ymm2, ymm1
	vsubpd ymm7, ymm3, ymm10
	vaddpd ymm2, ymm2, ymm0
	vsubpd ymm6, ymm2, ymm10
	vblendvpd ymm3, ymm7, ymm3, ymm7
	vblendvpd ymm2, ymm6, ymm2, ymm6
	vaddsd xmm7, xmm2, xmm2
	vsubsd xmm6, xmm7, xmm10
	vblendvpd xmm7, xmm6, xmm7, xmm6
	vmovapd [rdi], ymm0
	vmovq [rdi], xmm7
	vmovapd [rdi + 32], ymm1
	vmovapd [rdi + 64], ymm2
	vmovapd [rdi + 96], ymm3
	add rsi, waveform_size
	add rdi, 128
	dec ecx
	jnz .wavesetup

.mainloop:
	mov ebx, r12d
	cmp r12d, r14d
	cmovnc ebx, r14d
	lea ecx, [ebx + 15]
	shr ecx, 4
	mov rdi, [rbp + OutputBufferPointer]
.sampleloop:
	vxorpd xmm0, xmm0, xmm0
	vxorpd xmm1, xmm1, xmm1
	vxorpd xmm2, xmm2, xmm2
	vxorpd xmm3, xmm3, xmm3
	lea rax, [rbp + Waveforms]
	mov rsi, [rbp + WaveOffsetPointer]
.waveloop:
	vbroadcastsd ymm9, [rax + waveform.offset]
	vaddpd ymm4, ymm9, [rsi]
	vsubpd ymm8, ymm4, ymm10
	vaddpd ymm5, ymm9, [rsi + 32]
	vblendvpd ymm4, ymm8, ymm4, ymm8
	vsubpd ymm8, ymm5, ymm10
	vaddpd ymm6, ymm9, [rsi + 64]
	vblendvpd ymm5, ymm8, ymm5, ymm8
	vsubpd ymm8, ymm6, ymm10
	vaddpd ymm7, ymm9, [rsi + 96]
	vblendvpd ymm6, ymm8, ymm6, ymm8
	vsubpd ymm8, ymm7, ymm10
	vmovq [rax + waveform.offset], xmm4
	vblendvpd ymm7, ymm8, ymm7, ymm8
	vbroadcastsd ymm8, [rax + waveform.amplitude]
	vblendpd ymm4, ymm4, ymm9, 1
	%macro generatewave 1
		vmulpd ymm9, %1, ymm11
		vmovapd [rbp + SampleBuffer], ymm9
		vcmppd ymm9, %1, ymm15, CMP_NOT_EQ
		fld qword[rbp + SampleBuffer]
		fsin
		fstp qword[rbp + SampleBuffer]
		fld qword[rbp + SampleBuffer + 8]
		fsin
		fstp qword[rbp + SampleBuffer + 8]
		fld qword[rbp + SampleBuffer + 16]
		fsin
		fstp qword[rbp + SampleBuffer + 16]
		fld qword[rbp + SampleBuffer + 24]
		fsin
		fstp qword[rbp + SampleBuffer + 24]
		; special handling for mid-period values, because fsin is inaccurate around pi
		vblendvpd %1, ymm9, [rbp + SampleBuffer], ymm9
	%endmacro
	generatewave ymm5
	generatewave ymm4
	generatewave ymm6
	generatewave ymm7
	vmulpd ymm4, ymm4, ymm8
	vmulpd ymm5, ymm5, ymm8
	vmulpd ymm6, ymm6, ymm8
	vmulpd ymm7, ymm7, ymm8
	vaddpd ymm0, ymm0, ymm4
	vaddpd ymm1, ymm1, ymm5
	vaddpd ymm2, ymm2, ymm6
	vaddpd ymm3, ymm3, ymm7
	add rsi, 128
	add rax, waveform_size
	cmp rsi, [rbp + OutputBufferPointer]
	jc .waveloop
	vmulpd ymm0, ymm0, ymm14
	vmulpd ymm1, ymm1, ymm14
	vmulpd ymm2, ymm2, ymm14
	vmulpd ymm3, ymm3, ymm14
	vcmppd ymm4, ymm0, ymm12, CMP_LT
	vcmppd ymm5, ymm1, ymm12, CMP_LT
	vcmppd ymm6, ymm2, ymm12, CMP_LT
	vcmppd ymm7, ymm3, ymm12, CMP_LT
	vblendvpd ymm0, ymm12, ymm0, ymm4
	vblendvpd ymm1, ymm12, ymm1, ymm5
	vblendvpd ymm2, ymm12, ymm2, ymm6
	vblendvpd ymm3, ymm12, ymm3, ymm7
	vcmppd ymm4, ymm0, ymm13, CMP_GT
	vcmppd ymm5, ymm1, ymm13, CMP_GT
	vcmppd ymm6, ymm2, ymm13, CMP_GT
	vcmppd ymm7, ymm3, ymm13, CMP_GT
	vblendvpd ymm0, ymm13, ymm0, ymm4
	vblendvpd ymm1, ymm13, ymm1, ymm5
	vblendvpd ymm2, ymm13, ymm2, ymm6
	vblendvpd ymm3, ymm13, ymm3, ymm7
	call r13
	dec ecx
	jnz .sampleloop
	sub r12d, ebx
	mov cl, [rbp + OutputMode]
	shl ebx, cl
	call WriteOutput
	test r12d, r12d
	jnz .mainloop

	xor edi, edi
	mov eax, exit_group
	syscall
	ud2

WriteOutput:
	; don't write odd-sized buffers
	mov r15, [rbp + OutputBufferPointer]
	test bl, 1
	jz .go
	mov byte[r15 + rbx], 0
	inc ebx
.go:
	mov rsi, r15
	mov edx, ebx
	mov eax, write
	assert write == 1
	mov edi, eax
	syscall
	cmp rax, -EINTR
	jz .go
	lea rsi, [rel Messages.write_error]
	mov edx, Messages.write_error_end - Messages.write_error
	cmp rax, -0x1000
	jnc ErrorExit
	add r15, rax
	sub ebx, eax
	ja .go
	ret

SampleGenerators:
	; sample generators for each of the output modes
	; inputs: rdi: current output address; ymm0, ymm1, ymm2, ymm3: 16 double samples
	dq .bytes
	dq .words
	dq .floats

.bytes:
	endbr64
	mov rax, double(128.)
	mov [rbp + Temp], rax
	vbroadcastsd ymm9, [rbp + Temp]
	vaddpd ymm0, ymm0, ymm12
	vaddpd ymm1, ymm1, ymm12
	vaddpd ymm2, ymm2, ymm12
	vaddpd ymm3, ymm3, ymm12
	vaddpd ymm8, ymm9, ymm9
	vmulpd ymm4, ymm0, ymm9
	vmulpd ymm5, ymm1, ymm9
	vmulpd ymm6, ymm2, ymm9
	vmulpd ymm7, ymm3, ymm9
	vcmppd ymm0, ymm4, ymm8, CMP_GE
	vcmppd ymm1, ymm5, ymm8, CMP_GE
	vcmppd ymm2, ymm6, ymm8, CMP_GE
	vcmppd ymm3, ymm7, ymm8, CMP_GE
	vblendvpd ymm0, ymm0, ymm13, ymm0
	vblendvpd ymm1, ymm1, ymm13, ymm1
	vblendvpd ymm2, ymm2, ymm13, ymm2
	vblendvpd ymm3, ymm3, ymm13, ymm3
	vaddpd ymm0, ymm0, ymm4
	vaddpd ymm1, ymm1, ymm5
	vaddpd ymm2, ymm2, ymm6
	vaddpd ymm3, ymm3, ymm7
	vcvttpd2dq xmm4, ymm0
	vcvttpd2dq xmm5, ymm1
	vcvttpd2dq xmm6, ymm2
	vcvttpd2dq xmm7, ymm3
	vpshufb xmm0, xmm4, [rel .bytemasks]
	vpshufb xmm1, xmm5, [rel .bytemasks + 12]
	vpshufb xmm2, xmm6, [rel .bytemasks + 8]
	vpshufb xmm3, xmm7, [rel .bytemasks + 4]
	vpor xmm4, xmm0, xmm1
	vpor xmm5, xmm2, xmm3
	vpor xmm6, xmm4, xmm5
	vmovdqa [rdi], xmm6
	add rdi, 16
	ret

.words:
	endbr64
	mov rax, double(32768.)
	mov [rbp + Temp], rax
	vbroadcastsd ymm9, [rbp + Temp]
	vmulpd ymm4, ymm0, ymm9
	vmulpd ymm5, ymm1, ymm9
	vmulpd ymm6, ymm2, ymm9
	vmulpd ymm7, ymm3, ymm9
	vcmppd ymm0, ymm4, ymm9, CMP_GE
	vcmppd ymm1, ymm5, ymm9, CMP_GE
	vcmppd ymm2, ymm6, ymm9, CMP_GE
	vcmppd ymm3, ymm7, ymm9, CMP_GE
	vblendvpd ymm0, ymm0, ymm13, ymm0
	vblendvpd ymm1, ymm1, ymm13, ymm1
	vblendvpd ymm2, ymm2, ymm13, ymm2
	vblendvpd ymm3, ymm3, ymm13, ymm3
	vaddpd ymm0, ymm0, ymm4
	vaddpd ymm1, ymm1, ymm5
	vaddpd ymm2, ymm2, ymm6
	vaddpd ymm3, ymm3, ymm7
	vldmxcsr [rbp + FPCWTruncating]
	vcvtpd2dq xmm4, ymm0
	vcvtpd2dq xmm5, ymm1
	vcvtpd2dq xmm6, ymm2
	vcvtpd2dq xmm7, ymm3
	vldmxcsr [rbp + FPControlWord]
	vmovdqu xmm2, [rel .wordmasks + 8]
	vmovdqu xmm3, [rel .wordmasks]
	vpshufb xmm4, xmm4, xmm2
	vpshufb xmm5, xmm5, xmm3
	vpshufb xmm6, xmm6, xmm2
	vpshufb xmm7, xmm7, xmm3
	vpor xmm0, xmm4, xmm5
	vpor xmm1, xmm6, xmm7
	vmovdqa [rdi], xmm0
	vmovdqa [rdi + 16], xmm1
	add rdi, 32
	ret

.bytemasks:
	db 0, 4, 8, 12
	times 12 db -1
	db 0, 4, 8, 12
.wordmasks:
	times 8 db -1 ; also part of byte masks
	db 0, 1, 4, 5, 8, 9, 12, 13
	times 8 db -1

.floats:
	endbr64
	vcvtpd2ps xmm4, ymm0
	vcvtpd2ps xmm5, ymm1
	vcvtpd2ps xmm6, ymm2
	vcvtpd2ps xmm7, ymm3
	vmovaps [rdi], xmm4
	vmovaps [rdi + 16], xmm5
	vmovaps [rdi + 32], xmm6
	vmovaps [rdi + 48], xmm7
	add rdi, 64
	ret

ProcessArgument:
	; input: rsi: argument; rdi: next (if needed); r12: next waveform pointer
	; returns carry if the next argument was consumed
	cmp byte[rsi], 0
	jz .invalid
	cmp byte[rsi], "-"
	jnz .wave
	; all options are single character; fail on different lengths
	movzx eax, byte[rsi + 1]
	test al, al
	jz .invalid
	cmp byte[rsi + 2], 0
	jnz .invalid
	; find the option that was selected
	vmovd xmm0, eax
	mov rax, "bwqflsvh"
	vmovq xmm1, rax
	vpcmpistri xmm0, xmm1, 0
	jnc .invalid
	cmp cl, 6
	jz .volume
	ja .usage
	cmp cl, 3
	jz .frequency
	ja .length
	bts word[rbp + ArgumentFlags], 2
	lea rsi, [rel Messages.mode_already_set]
	mov edx, Messages.mode_already_set_end - Messages.mode_already_set
	jc ErrorExit
	; carry is clear here!
	mov [rbp + OutputMode], cl
	ret

.usage:
	lea rsi, [rel Messages.usage_1]
	mov rdi, [rbp + OutputBufferPointer]
	mov ecx, Messages.usage_2 - Messages.usage_1
	rep movsb
	mov rax, r15
	call StringLength
	mov ecx, 0x20000 - (Messages.usage_end - Messages.usage_1)
	cmp rdx, rcx
	cmovc ecx, edx
	mov rsi, r15
	rep movsb
	lea rsi, [rel Messages.usage_2]
	mov ecx, Messages.usage_end - Messages.usage_2
	jmp .move_buffer_error_exit

.invalid:
	mov rax, rsi
	lea rsi, [rel Messages.invalid_option_1]
	mov rdi, [rbp + OutputBufferPointer]
	mov ebx, 0x20000 - (Messages.invalid_option_end - Messages.invalid_option_1)
	mov ecx, Messages.invalid_option_2 - Messages.invalid_option_1
	rep movsb
	call StringLength
	cmp rdx, rbx
	mov ecx, edx
	cmovnc ecx, ebx
	sub ebx, ecx
	mov rsi, rax
	rep movsb
	lea rsi, [rel Messages.invalid_option_2]
	mov ecx, Messages.invalid_option_3 - Messages.invalid_option_2
	rep movsb
	mov rax, r15
	call StringLength
	cmp rdx, rbx
	mov ecx, edx
	cmovnc ecx, ebx
	mov rsi, r15
	rep movsb
	lea rsi, [rel Messages.invalid_option_3]
	mov ecx, Messages.invalid_option_end - Messages.invalid_option_3
.move_buffer_error_exit:
	rep movsb
.buffer_error_exit:
	mov rsi, [rbp + OutputBufferPointer]
	sub rdi, rsi
	mov edx, edi
	jmp ErrorExit

.frequency:
	bts word[rbp + ArgumentFlags], 0
	lea rsi, [rel Messages.frequency_already_set]
	mov edx, Messages.frequency_already_set_end - Messages.frequency_already_set
	jc ErrorExit
	test rdi, rdi
	lea rsi, [rel Messages.missing_frequency]
	mov edx, Messages.missing_frequency_end - Messages.missing_frequency
	jz ErrorExit
	mov rsi, rdi
	call ParseInteger
	jc .bad_frequency
	cmp byte[rsi], 0
	jnz .bad_frequency
	test rdx, rdx
	jz .bad_frequency
	cmp rdx, 1000000000
	jnc .bad_frequency
	; carry is set here!
	mov [rbp + OutputFrequency], edx
	ret

.bad_frequency:
	mov ebx, 0x1ffff - (Messages.bad_frequency_end - Messages.bad_frequency)
	lea rsi, [rel Messages.bad_frequency]
	mov ecx, Messages.bad_frequency_end - Messages.bad_frequency
.bad_argument:
	mov rax, rdi
	mov rdi, [rbp + OutputBufferPointer]
	rep movsb
	call StringLength
	cmp rdx, rbx
	mov ecx, edx
	cmovnc ecx, ebx
	mov rsi, rax
	rep movsb
	mov al, `\n`
	stosb
	jmp .buffer_error_exit

.length:
	bts word[rbp + ArgumentFlags], 1
	lea rsi, [rel Messages.length_already_set]
	mov edx, Messages.length_already_set_end - Messages.length_already_set
	jc ErrorExit
	test rdi, rdi
	lea rsi, [rel Messages.missing_length]
	mov edx, Messages.missing_length_end - Messages.missing_length
	jz ErrorExit
	mov rsi, rdi
	shr ecx, 1
	jc .samples
	call ParseNumber
	jc .bad_length
	cmp byte[rsi], 0
	jnz .bad_length
	mov rax, double(1000000000.)
	vmovq xmm0, rax
	vucomisd xmm1, xmm0
	jnc .bad_length
	vxorpd xmm0, xmm0, xmm0
	vucomisd xmm0, xmm1
	jnc .bad_length
	; carry is set here!
	vmovq [rbp + OutputLength], xmm1
	ret

.samples:
	call ParseInteger
	jc .bad_length
	cmp byte[rsi], 0
	jnz .bad_length
	or byte[rbp + ArgumentFlags + 1], 0x80
	test rdx, rdx
	jz .bad_length
	cmp rdx, 1000000000
	jnc .bad_length
	; carry is set here!
	mov [rbp + OutputLength], rdx
	ret

.bad_length:
	mov ebx, 0x1ffff - (Messages.bad_length_end - Messages.bad_length)
	lea rsi, [rel Messages.bad_length]
	mov ecx, Messages.bad_length_end - Messages.bad_length
	jmp .bad_argument

.volume:
	bts word[rbp + ArgumentFlags], 3
	lea rsi, [rel Messages.volume_already_set]
	mov edx, Messages.volume_already_set_end - Messages.volume_already_set
	jc ErrorExit
	test rdi, rdi
	lea rsi, [rel Messages.missing_volume]
	mov edx, Messages.missing_volume_end - Messages.missing_volume
	jz ErrorExit
	lea rsi, [rdi + 1]
	cmp byte[rdi], "-"
	cmovnz rsi, rdi
	call ParseNumber
	jc .bad_volume
	cmp byte[rsi], 0
	jnz .bad_volume
	mov rax, double(1000.)
	vmovq xmm0, rax
	vucomisd xmm1, xmm0
	jnc .bad_volume
	vxorpd xmm0, xmm0, xmm0
	vucomisd xmm0, xmm1
	jnc .bad_volume
	; carry is set here!
	vmovq [rbp + OutputVolume], xmm1
	ret

.bad_volume:
	mov ebx, 0x1ffff - (Messages.bad_volume_end - Messages.bad_volume)
	lea rsi, [rel Messages.bad_volume]
	mov ecx, Messages.bad_volume_end - Messages.bad_volume
	jmp .bad_argument

.wave:
	mov rdi, rsi
	call ParseNumber
	jc .bad_wave
	mov rax, double(1000000000.)
	vmovq xmm0, rax
	vucomisd xmm1, xmm0
	jnc .bad_wave
	vxorpd xmm0, xmm0, xmm0
	vucomisd xmm0, xmm1
	jnc .bad_wave
	vmovq xmm1, xmm1 ; clear upper double
	assert waveform.phase == waveform.frequency + 8
	vmovapd [r12 + waveform.frequency], xmm1
	mov rax, double(1.)
	mov [r12 + waveform.amplitude], rax
	xor ebx, ebx
.waveloop:
	lodsb
	test al, al
	jz .wavedone
	mov cl, al
	mov rdx, rsi
.scanloop:
	lodsb
	sub al, "0"
	cmp al, 10
	jc .scanloop
	mov rsi, rdx
	cmp al, "." - "0"
	jz .loadint
	call ParseFraction
	jmp .loaded
.loadint:
	call ParseNumber
	vmovapd xmm0, xmm1
.loaded:
	jc .bad_wave
	vxorpd xmm1, xmm1, xmm1
	vucomisd xmm0, xmm1
	jc .bad_wave
	cmp cl, "a"
	jz .waveamp
	cmp cl, "p"
	jnz .bad_wave
	bts ebx, 0
	jc .bad_wave
	vmovq [r12 + waveform.phase], xmm0
	jmp .waveloop

.waveamp:
	mov rax, double(100000.)
	vmovq xmm1, rax
	vucomisd xmm0, xmm1
	jnc .bad_wave
	vmovq [r12 + waveform.amplitude], xmm0
	jmp .waveloop

.wavedone:
	inc dword[rbp + WaveformCount]
	add r12, waveform_size ; clears carry unless r12 is cursed
	ret

.bad_wave:
	mov ebx, 0x1ffff - (Messages.bad_wave_end - Messages.bad_wave)
	lea rsi, [rel Messages.bad_wave]
	mov ecx, Messages.bad_wave_end - Messages.bad_wave
	jmp .bad_argument

StringLength:
	; input: rax: string; output: rdx: length
	lea rdx, [rax - 1]
.loop:
	inc rdx
	cmp byte[rdx], 0
	jnz .loop
	sub rdx, rax
	ret

ParseNumber:
	; input: rsi: number; output: xmm1: value, rsi: pointer to first non-digit, carry: overflow
	; clobbers rdx and xmm0; will parse a fractional part if present
	call ParseInteger
	jc .done
	vcvtsi2sd xmm1, xmm1, rdx
	cmp byte[rsi], "."
	clc
	jnz .done
	inc rsi
	call ParseFraction
	vaddsd xmm1, xmm1, xmm0 ; does not affect carry
.done:
	ret

ParseInteger:
	; input: rsi: integer; output: rdx: value, rsi: pointer to first non-digit, carry: overflow
	; treat empty strings as zero for simplicity; parse one digit at a time because reading ahead is not safe
	xor edx, edx
.loop:
	lodsb
	sub al, "0"
	cmp al, 10
	jnc .done
	lea rdx, [rdx + rdx * 4]
	add rdx, rdx
	jc .done
	movzx eax, al
	add rdx, rax
	jnc .loop
.done:
	dec rsi
	ret

ParseFraction:
	; input: rsi: fractional portion of number; output: xmm0: value, rsi: pointer to first non-digit, carry: digit overflow
	mov word[rbp + Temp], 5
	fild word[rbp + Temp]
	xor edx, edx
	xor eax, eax
	fld1
	fld1
	fldz
.loop:
	lodsb
	sub al, "0"
	cmp al, 10
	jnc .done
	mov [rbp + Temp], al
	fmul st0, st3
	fscale
	fxch st2
	fmul st0, st3
	fxch st2
	fild word[rbp + Temp]
	faddp st1
	dec edx
	cmp edx, -0x1000 ; limit the number of digits to avoid overflowing the exponent
	jnc .loop
.done:
	fstp st3
	fstp st0
	fdivp st1
	mov [rbp + Temp], edx
	fild dword[rbp + Temp]
	fxch st1
	fscale
	fstp st1
	fstp qword[rbp + Temp]
	vmovq xmm0, [rbp + Temp]
	dec rsi
	ret

Messages:
.no_stdout: withend db `error: no standard output\n`
.allocation_failure: withend db `error: failed to allocate memory\n`
.write_error: withend db `error: failed to write to standard output\n`
.bad_frequency_length: withend db `error: invalid output frequency and length combination\n`
.bad_frequency: withend db "error: invalid output frequency: "
.bad_length: withend db "error: invalid output length: "
.bad_volume: withend db "error: invalid output volume: "
.bad_wave: withend db "error: invalid wave declaration: "
.missing_frequency: withend db `error: missing output frequency\n`
.missing_length: withend db `error: missing output length\n`
.missing_volume: withend db `error: missing output volume\n`
.mode_already_set: withend db `error: output mode already set\n`
.frequency_already_set: withend db `error: output frequency already set\n`
.length_already_set: withend db `error: output length already set\n`
.volume_already_set: withend db `error: volume already set\n`

.invalid_option_1: db `error: invalid option "`
.invalid_option_2: db `". Run "`
.invalid_option_3: db ` -h" for help.\n`
.invalid_option_end:

.usage_1:
	db `sinewav - pure sine wave WAV generator\n\n`
.usage_2:
	db ` [options] wave [wave...]\n\n`
	db `\t-b:      output 8-bit PCM data\n`
	db `\t-w:      output 16-bit PCM data (default)\n`
	db `\t-q:      output 32-bit float samples\n`
	db `\t-f freq: set output frequency, in Hz (default: 44100)\n`
	db `\t-l len:  set output length, in seconds (default: 5)\n`
	db `\t-s len:  set output length, in samples\n`
	db `\t-v vol:  set output volume, in dB (default: 0)\n`
	db `\t-h:      show this help\n\n`
	db `Waves are specified as frequencies, potentially followed by some combination\n`
	db `of 'a' and an amplitude or 'p' and an initial phase (0 to 1). For example,\n`
	db `"480", "460.5", "400a0.7" and "300p0.5" are all valid waves.\n`
	db `As the amplitude and phase are almost always values from 0 to 1, the initial\n`
	db `"0." may be omitted: if the value contains no periods, it is assumed to be the\n`
	db `fractional portion alone. (For instance, "240a25" and "240a0.25" are the same.)\n`
	db `The volume, if set, cannot be positive; the argument is assumed to be negative,\n`
	db `and the initial minus sign may be omitted.\n`
	db `The arguments to -f and -s must be integers; all other numeric arguments may\n`
	db `contain fractional portions.\n`
.usage_end:

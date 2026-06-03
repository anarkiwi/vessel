; Shared Vessel user-port routines, used by the example programs.
;
; The C64 talks to Vessel over the user port (CIA2, port B):
;   $dd00 bit 2 : data direction. 1 = C64 -> Vessel (output), 0 = Vessel -> C64
;                 (input). The C64 always chooses the direction.
;   $dd01       : the 8 data bits PB0-7.
;   $dd03       : port B data direction register ($ff = output, $00 = input).
;
; See ../docs/API.md for the command set.

V_DATA = $dd01 ; user port data (PB0-7)
V_CTL = $dd00  ; CIA2 port A; bit 2 = direction
V_DDR = $dd03  ; port B data direction register
V_CMD = $fd    ; Vessel command marker (never a real MIDI byte)

; Switch the port to OUTPUT mode: the C64 sends MIDI / commands to Vessel.
vessel_output_mode:
        lda V_CTL
        ora #%00000100
        sta V_CTL
        lda #$ff
        sta V_DDR
        rts

; Switch the port to INPUT mode: the C64 reads MIDI buffered by Vessel.
vessel_input_mode:
        lda V_CTL
        and #%11111011
        sta V_CTL
        lda #$00
        sta V_DDR
        rts

; Read any MIDI Vessel has buffered into rx_buf; rx_count = number of bytes
; (0..255). Returns with the port back in output mode.
vessel_read:
        jsr vessel_input_mode
        ldx V_DATA ; first byte is the count Vessel will send
        stx rx_count
        beq .done
        ldy #0
.rx     lda V_DATA
        sta rx_buf,y
        iny
        dex
        bne .rx
.done   jmp vessel_output_mode ; tail call: its rts returns to our caller

rx_count:
        !byte 0
rx_buf:
        !fill 256, 0

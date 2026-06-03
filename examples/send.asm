; send - transmit MIDI through Vessel.
;
; Demonstrates "outputting MIDI": in output mode, bytes written to the port are
; transmitted on MIDI OUT. This plays middle C (note $3c) on channel 1 over and
; over, note on then note off, with a delay between.
;
; Build:  make send.prg      Run: LOAD"SEND.PRG",8,1 then RUN

!cpu 6510
!to "send.prg", cbm

* = $0801
        ; BASIC stub: 10 SYS 2061
        !byte $0b, $08, $0a, $00, $9e
        !text "2061"
        !byte $00, $00, $00

* = $080d
start:
        sei
        jsr vessel_output_mode

loop:
        ; Note On: channel 1 ($90), note 60 ($3c), velocity 100 ($64).
        lda #$90
        sta V_DATA
        lda #$3c
        sta V_DATA
        lda #$64
        sta V_DATA
        jsr delay

        ; Note Off: channel 1 ($80), note 60 ($3c), velocity 0.
        lda #$80
        sta V_DATA
        lda #$3c
        sta V_DATA
        lda #$00
        sta V_DATA
        jsr delay

        jmp loop

; Crude busy-wait so notes are audible.
delay:
        ldy #$60
.dy     ldx #$00
.dx     dex
        bne .dx
        dey
        bne .dy
        rts

!source "vessel.inc"

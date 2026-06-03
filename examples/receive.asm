; receive - configure Vessel and display incoming MIDI.
;
; Demonstrates "configuration" and "inputting MIDI": Vessel masks all MIDI by
; default, so this first configures it to forward note on/off + control change
; on all 16 channels plus all real-time messages, then polls and shows the most
; recent received bytes at the top of the screen.
;
; Build:  make receive.prg   Run: LOAD"RECEIVE.PRG",8,1 then RUN
;
; For NMI-on-MIDI-clock sync (instead of polling) see
; ../test/sidwizard/PROTOCOL.md.

!cpu 6510
!to "receive.prg", cbm

* = $0801
        ; BASIC stub: 10 SYS 2061
        !byte $0b, $08, $0a, $00, $9e
        !text "2061"
        !byte $00, $00, $00

* = $080d
start:
        sei
        jsr clear_screen
        jsr vessel_output_mode
        jsr vessel_configure

loop:
        jsr vessel_read
        ldx rx_count
        beq loop
        ldy #0
.show   lda rx_buf,y
        sta $0400,y      ; display received bytes (raw screen codes)
        iny
        dex
        bne .show
        jmp loop

; Reset, then enable all channels, all commands, and all real-time messages.
vessel_configure:
        ; Reset to defaults: command $00.
        lda #V_CMD
        sta V_DATA
        lda #$00
        sta V_DATA

        ; Channel mask: all 16 channels. Command $05, HH=$ff, LL=$ff.
        lda #V_CMD
        sta V_DATA
        lda #$05
        sta V_DATA
        lda #$ff
        sta V_DATA
        lda #$ff
        sta V_DATA

        ; Control mask: all commands on every channel. Command $07, once per
        ; channel, data $70..$7f (high nibble 7 = all commands, low nibble =
        ; channel 0..15).
        lda #$70
.cm     ldx #V_CMD
        stx V_DATA
        ldx #$07
        stx V_DATA
        sta V_DATA
        clc
        adc #$01
        cmp #$80
        bne .cm

        ; Status mask: all real-time / channel-less messages. Command $06,
        ; HH=$ff, LL=$ff.
        lda #V_CMD
        sta V_DATA
        lda #$06
        sta V_DATA
        lda #$ff
        sta V_DATA
        lda #$ff
        sta V_DATA
        rts

clear_screen:
        lda #$20
        ldx #0
.cs     sta $0400,x
        sta $0500,x
        sta $0600,x
        sta $0700,x
        inx
        bne .cs
        rts

!source "vessel.inc"

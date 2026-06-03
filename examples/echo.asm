; echo - MIDI loopback through Vessel.
;
; Enables Vessel's transparent mode (raw MIDI passthrough, no parsing or
; filtering) and echoes every received MIDI byte straight back out, so anything
; arriving on MIDI IN is re-transmitted on MIDI OUT. Received bytes are also
; written to the top of the screen as feedback.
;
; Build:  make echo.prg     Run: LOAD"ECHO.PRG",8,1 then RUN
;
; Originally a DefMON test program; rewritten for current Vessel firmware (which
; masks all MIDI by default, so transparent mode must be enabled explicitly).

!cpu 6510
!to "echo.prg", cbm

* = $0801
        ; BASIC stub: 10 SYS 2061
        !byte $0b, $08, $0a, $00, $9e
        !text "2061"
        !byte $00, $00, $00

* = $080d
start:
        sei
        jsr vessel_output_mode

        ; Enable transparent mode: command $04 (Config), flags $04 (bit 2).
        lda #V_CMD
        sta V_DATA
        lda #$04
        sta V_DATA
        lda #$04
        sta V_DATA

loop:
        jsr vessel_read
        ldx rx_count
        beq loop

        ; Echo the received bytes back out (already in output mode), and show
        ; them on screen.
        ldy #0
.echo   lda rx_buf,y
        sta V_DATA       ; re-transmit on MIDI OUT
        sta $0400,y      ; and display
        iny
        dex
        bne .echo
        jmp loop

!source "vessel.inc"

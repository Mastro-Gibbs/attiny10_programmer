# attiny10_programmer
---

### Description

This software allows you to program ATtiny family MCUs.
The supported MCUs are:
- ATtiny4
- ATtiny5
- ATtiny9
- ATtiny10

Other ATtiny family MCUs with possible support are:
- ATtiny20
- ATtiny40

---

### How to

This software is meant to run from an Arduino UNO.
Through a serial terminal (TeraTerm for example) you can interact with the text menu provided by the software, which allows you to interact with the ATtiny target with simple steps.

The supported commands are:
- 'W' | Enter programming mode
- 'I' | Read device ID
- 'R' | Read program via UART
- 'O' | Show received program
- 'P' | Program device
- 'V' | Verify flash
- 'D' | Dump memory
- 'E' | Erase chip (done automatically by 'P')
- 'X' | Exit programming mode
- 'F' | Show fuse
- 'S' | Set fuse menu
- 'C' | Clear fuse menu


import sys
sys.path.append('.')
import getkey

while True:
    ch = None
    while ch is None:
        print("press key")
        ch = getkey.getkey()
    try:
        chmap = next(k.name for k in reversed(getkey.Keys) if k==ch)
    except:
        chmap = chr(ch)

    print("CH",ch,chmap)
    if ch == getkey.Keys.ESC:
       break

    #ch = ch.upper()
    #if ch == 'ESC':
    #   break

import sys
sys.path.append('.')
from getkey import getkey, Keys

while True:
    ch = None
    while ch is None:
        print("press key")
        ch = getkey()
    try:
        chmap = next(k.name for k in reversed(Keys) if k==ch)
    except:
        chmap = chr(ch)

    print("CH",ch,chmap)
    if ch == Keys.ESC:
       break

    #ch = ch.upper()
    #if ch == 'ESC':
    #   break

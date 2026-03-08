class SignedWord:
    def __init__ (self, value=0, signed=True, bigendian=False):
        self.bigendian = bigendian
        self.signed = signed
        self.word = value & 0xFFFF
        if self.word >= (1<<15) and self.signed:
            self.word = self.word - 65536
        #if value:
        #    self.from_bytes(*self.lo_hi_bytes())
    def from_bytes (self, lo, hi):
        if self.bigendian:
            self.word = (hi & 0xFF) | ((lo & 0xFF) << 8)
        else:
            self.word = (lo & 0xFF) | ((hi & 0xFF) << 8)
        if self.word >= (1<<15) and self.signed:
            self.word = self.word - 65536
        return self
    def to_bytes (self):
        a, b = self.word & 0xFF, (self.word >> 8) & 0xFF
        if self.bigendian:
            return b, a
        else:
            return a, b
    def __int__ (self):
        return int(self.word)

def st3215_encode (word: int, signed: bool = True) -> tuple[int, int]:
    if word < 0 and not signed:
        raw = 65536 + word
    else:
        raw = word
    low = raw & 0xFF
    high = (raw >> 8) & 0xFF
    return low, high

def st3215_decode (lo: int, hi: int) -> int:
    return (lo&0xFF) | ((hi&0xFF)<<8)
def st3215_decode_signed (lo: int, hi: int) -> int:
    word = (lo&0xFF) | ((hi&0xFF)<<8)
    if word & 0x8000:
        return word - 65536
    return word 

pos = -100

print (SignedWord(pos).to_bytes())
print (st3215_encode(pos))
print (int(SignedWord(pos)))
print (int(SignedWord().from_bytes(156,255)))
print (int(SignedWord(65535,signed=False)))
print (st3215_decode_signed(*SignedWord(-32769).to_bytes()))
print (st3215_decode_signed(*SignedWord(pos).to_bytes()))

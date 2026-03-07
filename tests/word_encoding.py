class SignedWord:
    def __init__ (self, value=0, bigendian=False):
        self.bigendian = bigendian
        self.word = value
    def from_bytes (self, lo, hi):
        if self.bigendian:
            self.word = (hi & 0xFF) | ((lo & 0xFF) << 8)
        else:
            self.word = (lo & 0xFF) | ((hi & 0xFF) << 8)
        if self.word > (1<<15):
            self.word = self.word - 65536
        return self
    def lo_hi_bytes (self):
        a, b = self.word & 0xFF, (self.word >> 8) & 0xFF
        if self.bigendian:
            return b, a
        else:
            return a, b
    def __int__ (self):
        return int(self.word)

def st3215_decode (word):
    if word < 0:
        raw = 65536 + word
    else:
        raw = word
    low = raw & 0xFF
    high = (raw >> 8) & 0xFF
    return low, high

pos = -100

print (SignedWord(pos).lo_hi_bytes())
print (st3215_decode(pos))
print (int(SignedWord(pos)))
print (int(SignedWord().from_bytes(156,255)))

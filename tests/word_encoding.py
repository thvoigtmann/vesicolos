class Word16:
    def __init__ (self, value=0, signed=True, bitsigned=False, bigendian=False, safe_bound=False):
        """Represent a 16bit word.

        Parameters:
        -----------
        value : int
            Value stored in this 16-bit word.
        signed : bool (default: True)
            A signed word stores [-32768,32767], an unsigned word
            stores [0,65535].
        bitsigned : bool (default: False)
            A word with a sign bit, stores the sign in the most significant
            bit, accepts ranges [-32767,32767] with ambiguity at zero.
            Mutually exclusive with `signed`. If `bitsigned` is set to
            `True`, this takes precedence.
            Note that `bitsigned` and assignment with negative numbers
            gives strange results on machines that natively use the standard
            signed-word convention; in that case, it might be advisable to
            set `safe_bound=True` as well.
        bigendian : bool (default: False)
            A big-endian representation of the word returns the sequence
            `lo, hi` when `to_bytes()` is called.
            A little-endian representation returns `hi, lo`.
            Here, `lo` and `hi` are values in the range [0,255].
        safe_bound : bool (default: False)
            Determine how `value` is treated if it is outside the permissible
            range. If set to `False`, the value is taken `& 0xFFFF`, so
            that outside-range values wrap around. If set to `True`, the
            value is gently clipped to the maximum/minimum representable
            values in the 16-bit word (accounting for signednees).
        """
        self.bigendian = bigendian
        if bitsigned:
            signed = False
        self.__dict__['signed'] = signed
        self.__dict__['bitsigned'] = bitsigned
        self.safe_bound = safe_bound
        self._set_value (value)
    def _set_value (self, value):
        if not self.safe_bound:
            val = value & 0xFFFF
            if val >= (1<<15) and self.signed:
                val = val - 65536
            if self.bitsigned and (val & 0x8000):
                val = -(val & 0x7FFF)
        else:
            if self.signed:
                val = max(min(value,(1<<15)-1),-(1<<15))
            elif self.bitsigned:
                val = max(min(value,(1<<15)-1),1-(1<<15))
            else:
                val = min(value,(1<<16)-1)
        self.__dict__['value'] = val
    def __setattr__(self, name, value):
        """If `w` is a `Word16`, we allow `w.value = int` to set the value,
        representing the settings of safe clipping and signedness that
        `w` was created with. We also allow `w.value = a, b` setting the
        word value from a byte sequence, respecting the endianness set for
        `w`.

        If `w.signed` is assigned a new value, the stored values is converted
        in its signedness."""
        if name == 'value':
            if type(value) == tuple:
                self.from_bytes(*value)
            else:
                self._set_value(value)
        elif name == 'signed':
            self.__dict__[name] = value
            if self.signed:
                self.__dict__['bitsigned'] = False
            self.from_bytes(*self.to_bytes())
        elif name == 'bitsigned':
            self.__dict__[name] = value
            if self.bitsigned:
                self.__dict__['signed'] = False
            self.from_bytes(*self.to_bytes())
        else:
            self.__dict__[name] = value
    def from_bytes (self, a, b):
        """Assign the 16-bit value from a two-byte sequence.
        If `self` is a big-endian word, the values are `lo, hi`.
        If `self` is a little-endian word, the values are `hi, lo`."""
        if self.bigendian:
            self.value = (a & 0xFF) | ((b & 0xFF) << 8)
        else:
            self.value = (b & 0xFF) | ((a & 0xFF) << 8)
        if self.value >= (1<<15) and self.signed:
            self.value = self.value - 65536
        if self.bitsigned and (self.value & 0x8000):
            self.value = -(self.value & 0x7FFF)
        return self
    def to_bytes (self):
        """Return the value represented as a two-byte sequence.
        If `self` is big-endian, returns `lo, hi`, else `hi, lo`."""
        if not self.bitsigned:
            lo, hi = self.value & 0xFF, (self.value >> 8) & 0xFF
        else:
            lo, hi = abs(self.value) & 0xFF, (abs(self.value) >> 8) & 0x7F
            if self.value < 0:
                hi |= 0x80
        if self.bigendian:
            return lo, hi
        else:
            return hi, lo
    def swap_endianness (self):
        """Change the endianness of the word. Does not change the value,
        but the interpretation of the byte sequences in `from_bytes`
        and `to_bytes`."""
        self.bigendian = not self.bigendian
        return self
    def __int__ (self):
        return int(self.value)

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

#pos = -100

#print (Word16(pos).to_bytes())
#print (st3215_encode(pos))
#print (int(Word16(pos)))
#print (int(Word16(bigendian=True).from_bytes(156,255)))
#print (int(Word16(65535,signed=False)))
#print (st3215_decode_signed(*Word16(-32769,bigendian=True,safe_bound=True).to_bytes()))
#print (st3215_decode_signed(*Word16(pos,bigendian=True).to_bytes()))


class kvMessage(object):
    """Represents a CAN message

    Args:
        id_: Message id
        data : Message data, will pad zero to match dlc (if dlc is given)
        dlc : Message dlc, default is calculated from number of data
        flags : Message flags, default is 0
        timestamp : Optional timestamp
    """

    def __init__(self, id_, data, dlc=None, flags=0, timestamp=None):
        self.id = id_
        if dlc is None:
            dlc = len(data)
        self.dlc = dlc
        if dlc > len(data):
            data.extend([0] * (dlc - len(data)))
        if not isinstance(data, (bytes, str)):
            if not isinstance(data, bytearray):
                data = bytearray(data)
            data = bytes(data)
        self.data = data
        self.flags = flags
        self.timestamp = timestamp

    def __ne__(self, other):
        return not self.__eq__(other)

    def __eq__(self, other):
        if self.id != other.id:
            return False
        if self.dlc != other.dlc:
            return False
        if self.data != other.data:
            return False
        if self.flags != other.flags:
            return False
        return True

    def __str__(self):
        if self.timestamp is None:
            timestamp = "        -  "
        else:
            timestamp = "%11.3f" % (self.timestamp/1000.0)
        flags = self.flags
        id = self.id
        dlc = self.dlc
        data = " ".join("{:02x}".format(ord(c)) for c in self.data)
        text = ("t:%s ch:%s f:%5x id:%4x dlc:%2d d:%s" %
                (timestamp, "-", flags, id, dlc, data))
        return text

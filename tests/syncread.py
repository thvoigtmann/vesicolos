class Test:
    def readRx(self, rxpacket, scs_id, data_length):
        # print(scs_id)
        # print(rxpacket)
        data = []
        rx_length = len(rxpacket)
        # print(rx_length)
        rx_index = 0;
        while (rx_index+6+data_length) <= rx_length:
            headpacket = [0x00, 0x00, 0x00]
            while rx_index < rx_length:
                headpacket[2] = headpacket[1];
                headpacket[1] = headpacket[0];
                headpacket[0] = rxpacket[rx_index];
                rx_index += 1
                if (headpacket[2] == 0xFF) and (headpacket[1] == 0xFF) and headpacket[0] == scs_id:
                    # print(rx_index)
                    break
            # print(rx_index+3+data_length)
            if (rx_index+3+data_length) > rx_length:
                break;
            if rxpacket[rx_index] != (data_length+2):
                rx_index += 1
                # print(rx_index)
                continue
            rx_index += 1
            Error = rxpacket[rx_index]
            rx_index += 1
            calSum = scs_id + (data_length+2) + Error
            data = [Error]
            data.extend(rxpacket[rx_index : rx_index+data_length])
            for i in range(0, data_length):
                calSum += rxpacket[rx_index]
                rx_index += 1
            calSum = ~calSum & 0xFF
            # print(calSum)
            if calSum != rxpacket[rx_index]:
                return None, "COMM_RX_CORRUPT"
            return data, "COMM_SUCCESS"
        # print(rx_index)
        return None, "COMM_RX_CORRUPT"
    def parseRx (self, rxpacket, data_length):
        bytelist = iter(rxpacket)
        response = {}
        try:
            while next(bytelist)==0xFF and next(bytelist)==0xFF:
                scs_id = next(bytelist)
                retlen = next(bytelist)
                if retlen != data_length+2:
                    return None, "COMM_RX_CORRUPT"
                err = next(bytelist)
                data = [next(bytelist) for _ in range(data_length)]
                crc = ~(sum(data) + data_length + 2 + scs_id + err) & 0xFF
                rcrc = next(bytelist)
                if crc != rcrc:
                    return None, "COMM_RX_CORRUPT"
                response[scs_id] = data
                response[scs_id] = {
                        "header": [0xFF, 0xFF],
                        "id": scs_id,
                        "length": retlen,
                        "error": err,
                        "parameters": data,
                        "received_checksum": rcrc,
                        "calculated_checksum": crc,
                        "checksum_valid": (rcrc==crc)
                }
        except StopIteration:
            pass
        return response, "COMM_SUCCESS"




t = Test()
packet = [0xff, 0xff, 0x01 ,0x0a , 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x79, 0x1e, 0x55, 0xff, 0xff, 0x02, 0x0a, 0x00, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x77, 0x23, 0x53]
print (t.readRx(packet, 2, 8))
print (t.parseRx(packet, 8))

import struct
from enum import Enum
from typing import Tuple
import json
import time

from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian

class CPCOE_CHDIR(Enum):
    """
        Enumerator for setting the direction of universal channel
    """
    INPUT = 0
    """
        Output type
    """
    OUTPUT = 1
    """
        Input type
    """

class CPCOE_CHTYPE_OUT(Enum):
    """
        Enumerator for setting the type of output channel
    """
    OUT_0_10 = 0
    """
    0-10V analog output channel
    """
    OUT_PWM_100 = 3
    """
    PWM output with 100Hz
    """
    OUT_PWM_2 = 5
    """
    PWM output with 2kHz
    """

class CPCOE_CHTYPE_IN(Enum):
    """
        Enumerator for setting the type of input channel
    """
    NTC = 0
    """
        NTC temperature sensor
    """
    PT1000 = 1
    """
        PT1000 temperature sensor
    """
    ANALOG_0_1 = 2
    """
        Analog 0-1V input
    """
    ANALOG_0_10 = 3
    """
        Analog 0-10V input
    """
    ANALOG_4_20 = 4
    """
        Analog 4-20mA current loop input
    """
    DIN = 5
    """
        Digital input 
    """
    ANALOG_0_5 = 6
    """
        Analog 0-5V input
    """
    NTC_0_150 = 7
    """
        Enumerator for setting the type of input channel
    """
    NTC_TEMP_PRESS = 11
    """
        Enumerator for setting the type of input channel
    """
    PTC_R = 34
    """
        Enumerator for setting the type of input channel
    """
    PT500 = 35
    """
        Enumerator for setting the type of input channel
    """
    PT100 = 36
    """
        Enumerator for setting the type of input channel
    """
    ANALOG_0_20 = 42
    """
        Enumerator for setting the type of input channel
    """
    NOT_USED = 255
    """
        Enumerator for setting the type of input channel
    """


def holding32_to_real(lreg, rreg):

    data1 = lreg  
    data2 = rreg 
    data2 = data2 + (data1 << 16)
    res = data2.to_bytes(4, 'big')

    return round(struct.unpack('>f', res)[0], 2)

class ModbusDataChunk:
    def __init__(self, map_filepath:str, map_filename:str, client:ModbusSerialClient, dev_address:int):

        cpath = map_filepath + map_filename

        self.load_data_map_from_json(cpath)
        self.client = client
        self.dev_address = dev_address

        self.start = self.var_table[0]['addr']
        self.stop = self.var_table[-1]['addr']
        self.count = self.stop - self.start + 1

        self.read_regs, self.write_regs = self.bind_modbus_functions(client)        


        self.registers = self.encode_data_block(self.var_table)


    def load_data_map_from_json(self, cpath:str):

        data = {}
        with open(cpath, 'r', encoding='utf-8') as file:
            data = json.load(file)

        self.name = data['name']
        self.type = data['type']
        self.var_table = data['var_table']    
        

    def bind_modbus_functions(self, client:ModbusSerialClient):

        if self.type == 'holding':
            return client.read_holding_registers, client.write_registers
        elif self.type == 'input':
            return client.read_input_registers, None




    def encode_data_block(self, data):

        registers = []
        for reg in data:
            if reg['type'] == 'uint16' or reg['type'] == 'bool':
                registers.append(reg['value'])
                #print(reg['value'], '>H')
            elif reg['type'] == 'int16':
                registers.append(reg['value'])
                #print(reg['value'], '>h')
            elif reg['type'] == 'float32':
                data = int.from_bytes(struct.pack('>f', reg['value'])) 
                lreg = (data >> 16) & 0xFFFF
                rreg = (data) & 0xFFFF
                registers.append(lreg)
                registers.append(rreg)
            else:
                raise ValueError(f'unknown type {reg["type"]}')
        return registers


    def read_data_block(self, slave):
        resp = self.read_regs(self.start, self.count + 1, slave=slave)
        resp_data = resp.registers

        #print("REGISTERS:" , resp_data)
        i = 0
        for v in self.var_table:
            if v['len'] == 2 and v['type'] == "float32":
                #print(resp_data[i])
                #print(resp_data[i + 1])
                real = holding32_to_real(resp_data[i], resp_data[i+1])
                v['value'] = real
                #print(f'{v["name"]} ({v['addr']}): {real}')
                i += 2
            else:
                v['value'] = resp_data[i]
                i += 1


    def write_data_block(self, client:ModbusSerialClient, slave):
        self.registers = self.encode_data_block(self.var_table)
        self.client.write_registers(self.start, self.registers, slave=slave)
        pass


    def set_var_by_name(self, var_name:str, value, save:bool=False):
        for v in self.var_table:
            if v['name'] == var_name:
                v['value'] = value
                if save:

                    if isinstance(v['value'], float):
                        builder = BinaryPayloadBuilder(
                            byteorder=Endian.BIG,
                            wordorder=Endian.BIG
                        )

                        builder.add_32bit_float(v['value'])
                        registers = builder.to_registers()
                        print(v['value'])
                    
                        self.write_regs(v['addr'], values=registers, slave=self.dev_address)
                    else:
                        self.write_regs(v['addr'], v['value'], self.dev_address)
                return v
            

        






class CPCOE_Device:
    def __init__(self, address, com_port, data_maps_path='devices/', baudrate=19200, bytesize=8, parity='N', stopbits=1):

        self.dev_address = address
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits

        self.data_chunks = []

        
    
        

        self.client = ModbusSerialClient(
            com_port,
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            stopbits=self.stopbits
        )

        self.client.connect()

        self.io_config = ModbusDataChunk('cpcoe/', 'cpcoe_io_config.json', self.client, self.dev_address)
        self.io_vals = ModbusDataChunk('cpcoe/', 'cpcoe_io_vals.json', self.client, self.dev_address)
        self.io_errors = ModbusDataChunk('cpcoe/', 'cpcoe_io_errors.json', self.client, self.dev_address)

        self.io_config.read_data_block(address)




    def config_univ_channel(self, chnum:int, chdir:CPCOE_CHDIR, chtype_out:CPCOE_CHTYPE_OUT=0, chtype_in:CPCOE_CHTYPE_IN=0, actPrbRange:Tuple[float, float]=(0.0, 0.0), filterSamples:int=5, save:bool=False):

        chname = 'UnivChs[' + str(chnum) + ']'
        if chdir == CPCOE_CHDIR.INPUT:
            self.io_config.set_var_by_name(chname + '.ChDir', chdir.value)
            self.io_config.set_var_by_name(chname + '.ChTyp', chtype_in.value)

        elif chdir == CPCOE_CHDIR.OUTPUT:
            self.io_config.set_var_by_name(chname + '.ChDir', chdir.value)
            self.io_config.set_var_by_name(chname + '.ChTyp', chtype_out.value)


        self.io_config.set_var_by_name(chname + '.ActPrbMin', actPrbRange[0])
        self.io_config.set_var_by_name(chname + '.ActPrbMax', actPrbRange[1])  
        self.io_config.set_var_by_name(chname + '.FiltSamplesNo', filterSamples)  
        self.io_config.set_var_by_name(chname + '.UpdThrsh', 0.0)  

        if save:
            self.io_config.write_data_block(self.client, self.dev_address)


    def write_data_chunk():
        pass


    def read_values(self):
        self.io_vals.read_data_block(self.dev_address)
        return self.io_vals.var_table
    


    def set_digital_output(self, output_num:int, value:bool, save:bool=False):
        """
            Set digital output channel state

            Args:
                output_num (int): output id
                value (bool): output state
                save (bool)(optional): save io device memory immediately after changing value

        """
        chname =  "DOutVals[" + str(output_num) + "]"
        self.io_vals.set_var_by_name(chname, value, save=save)

    def get_univ_ch_val(self, chnum:int):
        """
            Get the value from universal channel

            Args:
                chnum (int): channel number

        """
        chname =  "UnivChsVals[" + str(chnum) + "]"
        for v in self.io_vals.var_table:
            if v['name'] == chname:
                return v['value']
        return None 
    
    def set_univ_ch_val(self, chnum:int, value):
        """
            Set value of universal channel

            Args:
                chnum (int): channel number
                value : value to set

        """
        chname =  "UnivChsVals[" + str(chnum) + "]"
        self.io_vals.set_var_by_name(chname, value, save=True)


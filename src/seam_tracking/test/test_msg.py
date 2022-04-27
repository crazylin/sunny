# Copyright 2019 Zhushi Tech, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


def modbus_msg(id: str, valid: bool, u: float, v: float):
    id = int(id) % 0x10000
    # Transaction identifier
    s = id.to_bytes(2, 'big')
    # Protocol identifier 2, Length field 2, Unit identifier 1, Function code 1
    s += bytes([0x00, 0x00, 0x00, 0x0d, 0x01, 0x10])
    # Start address 2, number of registers, number of bytes
    s += bytes([0x00, 0x02, 0x00, 0x03, 0x06])

    b = bytes([0x00, 0xff]) if valid else bytes([0x00, 0x00])

    try:
        t = bytes()
        t += round(u * 100).to_bytes(2, 'big')
        t += round(v * 100).to_bytes(2, 'big')
    except Exception:
        s += bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    else:
        s += b + t

    return s


def test_modbus_msg():
    s = modbus_msg('123', True, 0.01, 0.02)
    assert s == bytes([
        0x00, 0x7b, 0x00, 0x00,
        0x00, 0x0d, 0x01, 0x10,
        0x00, 0x02, 0x00, 0x03,
        0x06, 0x00, 0xff, 0x00,
        0x01, 0x00, 0x02])

    s = modbus_msg('123', False, 0.01, 0.02)
    assert s == bytes([
        0x00, 0x7b, 0x00, 0x00,
        0x00, 0x0d, 0x01, 0x10,
        0x00, 0x02, 0x00, 0x03,
        0x06, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x02])

    s = modbus_msg('65537', False, 0.01, 0.02)
    assert s == bytes([
        0x00, 0x01, 0x00, 0x00,
        0x00, 0x0d, 0x01, 0x10,
        0x00, 0x02, 0x00, 0x03,
        0x06, 0x00, 0x00, 0x00,
        0x01, 0x00, 0x02])

    s = modbus_msg('65537', False, 655, 0.02)
    assert s == bytes([
        0x00, 0x01, 0x00, 0x00,
        0x00, 0x0d, 0x01, 0x10,
        0x00, 0x02, 0x00, 0x03,
        0x06, 0x00, 0x00, 0xff,
        0xdc, 0x00, 0x02])

    s = modbus_msg('65537', False, 656, 0.02)
    assert s == bytes([
        0x00, 0x01, 0x00, 0x00,
        0x00, 0x0d, 0x01, 0x10,
        0x00, 0x02, 0x00, 0x03,
        0x06, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00])

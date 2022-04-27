// Copyright 2019 Zhushi Tech, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <set>

#undef NDEBUG
#include <cassert>

int main()
{
  auto ctx = modbus_new_tcp(NULL, 2345);
  assert(ctx);

  // _mb_mapping = modbus_mapping_new(MODBUS_MAX_READ_BITS, 0, MODBUS_MAX_READ_REGISTERS, 0);
  auto mb_mapping = modbus_mapping_new(0, 0, 400, 0);
  assert(mb_mapping);
  mb_mapping->tab_registers[1] = 0xff;

  auto sock = modbus_tcp_listen(ctx, 10);
  assert(sock != -1);

  std::set<int> fds {sock};

  fd_set refset;
  FD_ZERO(&refset);
  FD_SET(sock, &refset);

  int fdmax = sock;
  uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
  int ret = 0;
  while (ret != -1) {
    auto rdset = refset;
    timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    ret = select(fdmax + 1, &rdset, NULL, NULL, &tv);
    assert(ret != -1);

    auto fds_bak = fds;
    for (auto fd : fds_bak) {
      if (!FD_ISSET(fd, &rdset)) {continue;}

      if (fd == sock) {
        // A client is asking a new connection
        ret = modbus_tcp_accept(ctx, &sock);
        if (ret != -1) {
          FD_SET(ret, &refset);
          fds.insert(fds.end(), ret);
          fdmax = *fds.rbegin();
        } else {
          assert(ret != -1);
        }
      } else {
        // A client is asking for reply
        ret = modbus_set_socket(ctx, fd);
        assert(ret != -1);
        ret = modbus_receive(ctx, query);
        if (ret == -1) {
          // Connection closed by the client or error
          close(fd);
          FD_CLR(fd, &refset);
          fds.erase(fd);
          fdmax = *fds.rbegin();
          ret = -1;
          break;
        } else if (ret > 0) {
          ret = modbus_reply(ctx, query, ret, mb_mapping);
          for (auto i = 2; i < 5; ++i) {
            assert(mb_mapping->tab_registers[i] == i - 1);
          }
          assert(ret != -1);
        }
      }
    }
  }

  close(sock);
  modbus_mapping_free(mb_mapping);
  modbus_free(ctx);
  return 0;
}

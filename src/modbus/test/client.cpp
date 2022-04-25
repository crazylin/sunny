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

#include <errno.h>
#include <modbus.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>     // inet_addr

#include <vector>

#undef NDEBUG
#include <cassert>

int main()
{
  std::vector<unsigned char> d = {0, 1, 0, 0, 0, 13, 1, 16, 0, 0, 0, 3, 6, 0, 1, 0, 2, 0, 3};

  int socket_desc;
  struct sockaddr_in server;
  // char *message , server_reply[2000];

  // Create socket
  socket_desc = socket(AF_INET, SOCK_STREAM, 0);
  assert(socket_desc != -1);

  server.sin_addr.s_addr = inet_addr("127.0.0.1");
  server.sin_family = AF_INET;
  server.sin_port = htons(2345);

  // Connect to remote server
  int ret = connect(socket_desc, (struct sockaddr *)&server, sizeof(server));
  assert(ret != -1);

  // Send some data
  // message = "GET / HTTP/1.1\r\n\r\n";
  assert(send(socket_desc, d.data(), d.size(), 0) == 19);
  // modbus_t * ctx = modbus_new_tcp("127.0.0.1", 2345);

  // auto sock = modbus_connect(ctx);

  // send(sock, d.data(), d.size(), 0);
  return 0;
}

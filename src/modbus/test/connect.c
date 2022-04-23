#include <errno.h>
#include <modbus.h>
#include <stdio.h>

int main()
{
  modbus_t * ctx = modbus_new_tcp("127.0.0.1", 2345);

  if (modbus_connect(ctx) == -1) {
    fprintf(
      stderr, "Connection failed: %s\n",
      modbus_strerror(errno));
    modbus_free(ctx);
    return -1;
  }

  getchar();
}
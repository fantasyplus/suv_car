#pragma once

int open_serial_port(const char *port_name, int baud_rate, int bits, char event, int stop);

int read_nbyte(int fd, char *buf_ptr, const int nbytes);

int read_until(int fd, char *buf_ptr, const int buf_len, const char *delim);
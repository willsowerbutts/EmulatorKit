#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include "ide.h"

int main(int argc, const char *argv[])
{
  int t, fd, sparse=0;

  if (argc != 3) {
    fprintf(stderr, "%s [type] [path]\n\n", argv[0]);
    
    fprintf(stderr, "type 1:  512MB ACME ROADRUNNER                 1024C  16H   63S  LBA\n");
    fprintf(stderr, "type 2:   20MB ACME COYOTE                      615C   4H   16S\n");
    fprintf(stderr, "type 3:   20MB ACME NEMESIS RIDICULII           615C   4H   16S  LBA\n");
    fprintf(stderr, "type 4:   40MB ACME ULTRASONICUS AD INFINITUM   977C   5H   16S  LBA\n");
    fprintf(stderr, "type 5:  128MB ACME ACCELLERATTI INCREDIBILUS  1024C  16H   16S  LBA\n");
    fprintf(stderr, "type 6:  256MB ACME ZIPPIBUS                   1024C  16H   32S  LBA\n");
    fprintf(stderr, "type 7: 8192MB ACME BIGGUS DISKUS              8192C  16H  128S  LBA\n\n");

    fprintf(stderr, "Entering a negative type number will create a sparse file.\n");
    fprintf(stderr, "Sparse file disk images are initialised with all 0x00 bytes.\n");
    fprintf(stderr, "Non-sparse file disk images are initialised with all 0xE5 bytes.\n");

    exit(1);
  }

  t = atoi(argv[1]);
  if(t<0) {
      sparse=1;
      t = -t;
  }
  if (t < 1 || t > MAX_DRIVE_TYPE) {
    fprintf(stderr, "%s: unknown drive type.\n", argv[0]);
    exit(1);
  }
  fd = open(argv[2], O_WRONLY|O_TRUNC|O_CREAT|O_EXCL, 0666);
  if (fd == -1) {
    perror(argv[2]);
    exit(1);
  }
  if (ide_make_drive(t, fd, sparse) < 0) {
    perror(argv[2]);
    exit(1);
  }
  return 0;
}

/* 
  Use this program to make a file of cliff thresholds (cliffThresholds.dat).

  WARNING: this program will overwrite the file cliffThresholds.dat!

  It takes as input two files, each a calibration of the cliff sensors
  on a particular robot on a particular surface, of the format made by
  the program "cliffcalib". The first file corresponds to the inside
  surface and the second to the outer surface.

  Usage: 

       makethresh insidefile outsidefile

  where insidefile defaults to "inside" and outsidefile defaults to "outside".
  makethresh creates the file cliffThesholds.dat needed by programs that constrain
  the create robot to limited uniform colored regions.

  This program was originally written by Rich Sutton (rich@richsutton.com) 
  in June, 2013.
*/

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define REQUIRED_MARGIN 100
#define MIN(a,b) (a < b?a:b)

int main(int argc, char *argv[]) {
  int minMargin, i;
  int insideMin[4], outsideMin[4];
  int insideMax[4], outsideMax[4];
  FILE *fd;
  char *insidefile, *outsidefile;

  if (argc==3) {
    insidefile = argv[1];
    outsidefile = argv[2];
  } else if (argc==1) {
    insidefile = "inside";
    outsidefile = "outside";
  }

  // INSIDE
  if ((fd = fopen(insidefile, "r"))==NULL) {
    printf("Error opening inside file: %s, error: %s\n", 
	   insidefile, strerror(errno));
    exit(EXIT_FAILURE);
  }
  for (i = 0; i < 4; i++) fscanf(fd, "%d%d", &insideMin[i], &insideMax[i]);
  fclose(fd);

  // OUTSIDE
  if ((fd = fopen(outsidefile, "r"))==NULL) {
    printf("Error opening outside file: %s, error: %s\n", 
	   outsidefile, strerror(errno));
    exit(EXIT_FAILURE);
  }
  for (i = 0; i < 4; i++) fscanf(fd, "%d%d", &outsideMin[i], &outsideMax[i]);
  fclose(fd);

  // inside lower/darker
  minMargin = 5000;
  for (i = 0; i < 4; i++)
    minMargin = MIN(minMargin, outsideMin[i] - insideMax[i]);
  if (minMargin > REQUIRED_MARGIN) {
    if ((fd = fopen("cliffThresholds.dat", "w"))==NULL) {
      printf("Error opening cliffThresholds.dat file: %s\n", strerror(errno));
      exit(EXIT_FAILURE);
    }
    fprintf(fd, "1\n");
    for (i = 0; i < 4; i++) fprintf(fd, "%d\n", (outsideMin[i]+insideMax[i])/2);
    fclose(fd);
    printf("Success! Margin is %d (inside is darker/lower)\n", minMargin);
    return 0;
  }
  // outside lower/darker
  minMargin = 5000;
  for (i = 0; i < 4; i++)
    minMargin = MIN(minMargin, insideMin[i] - outsideMax[i]);
  if (minMargin > REQUIRED_MARGIN) {
    if ((fd = fopen("cliffThresholds.dat", "w"))==NULL) {
      printf("Error opening cliffThresholds.dat file: %s\n", strerror(errno));
      exit(EXIT_FAILURE);
    }
    fprintf(fd, "0\n");
    for (i = 0; i < 4; i++) fprintf(fd, "%d\n", (insideMin[i]+outsideMax[i])/2);
    printf("Success! Margin is %d (inside is lighter/higher)\n", minMargin);
    fclose(fd);
    return 0;
  }
  printf("Failure -- could not find acceptable thresholds\n");
  if (minMargin > 0) 
    printf("Margin %d too small; needs to be %d\n", minMargin, REQUIRED_MARGIN);
  else
    printf("Bounds overlap by %d\n", -minMargin);
}

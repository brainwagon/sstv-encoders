#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <math.h>
#include <fftw3.h>
#include <sndfile.h>

float 
window(int n, int N)
{
   /* Blackman-Nutall Window */
   float a0 = 0.3635819 ;
   float a1 = 0.4891775 ;
   float a2 = 0.1365995 ;
   float a3 = 0.0106411 ;

   return a0 - a1 * cos(2.0*M_PI*n/(double)(N-1))
	     + a2 * cos(4.0*M_PI*n/(double)(N-1))
	     + a3 * cos(6.0*M_PI*n/(double)(N-1)) ;
}

int
main(int argc, char *argv[])
{
    SNDFILE *sf ;
    SF_INFO sfinfo ;
    float *inp ;
    float t, dt ;
    float x ;
    int i, j ;
    float pcross=0, cross, freq ;
    fftw_complex *ip, *op ;
    fftw_plan p ;

    if ((sf = sf_open(argv[1], SFM_READ, &sfinfo)) == NULL) {
	perror(argv[1]) ;
	exit(1) ;
    }
 
    fprintf(stderr, "%s: %d channel%s\n", argv[1], sfinfo.channels, 
		sfinfo.channels > 1 ? "s" : "") ;
    fprintf(stderr, "%s: %dHz\n", argv[1], sfinfo.samplerate) ;
    fprintf(stderr, "%s: %lld samples\n", argv[1], sfinfo.frames) ;

    inp = (float *) calloc(sfinfo.frames, sizeof(float)) ;
    fprintf(stderr, "::: reading %lld frames\n", sfinfo.frames) ;
    sf_read_float(sf, inp, sfinfo.frames) ;
    sf_close(sf) ;

    ip = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * sfinfo.frames) ;
    op = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * sfinfo.frames) ;

    p = fftw_plan_dft_1d(sfinfo.frames, ip, op, FFTW_FORWARD, FFTW_ESTIMATE) ;

    for (i=0; i<sfinfo.frames; i++)
	ip[i] = inp[i] * window(i, sfinfo.frames) ;

    fftw_execute(p) ;
    fftw_destroy_plan(p) ;

#define BOXFILTER	100
    for (i=0; i<sfinfo.frames/2-BOXFILTER; i++) {
	double sum = 0. ;
	for (j=0; j<BOXFILTER; j++)
	    sum += cabs(op[i]) ;
	printf("%lf %lf\n", i * 11025. / sfinfo.frames, sum) ;
    }

    fftw_free(ip) ; fftw_free(op) ;

}

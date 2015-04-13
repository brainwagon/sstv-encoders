#include <stdio.h>
#include <stdlib.h>
#include <complex.h>
#include <math.h>
#include <setjmp.h>
#include <assert.h>

/* 
 * $Id$
 *
 *  ___  ___  ___  ___ _____ ____ ___ 
 * | _ \/ _ \| _ )/ _ \_   _|__  |_  )
 * |   / (_) | _ \ (_) || |   / / / / 
 * |_|_\\___/|___/\___/ |_|  /_/ /___|
 *                                    
 * An implementation of a Robot 72 SSTV encoder
 * Written by Mark VandeWettering (K6HX)
 */

/* JPEG related headers */
#include <jpeglib.h>
#include <jerror.h>

/* soundfile related headers */
#include <sndfile.h>

float
grand()
{
    static int flag = 0 ;
    static float y1, y2 ;
    float x1, x2, w ;
 
    if (flag) {
	flag = 0 ;
	return y2 ;
    }
    do {
	x1 = 2.0 * drand48() - 1.0;
	x2 = 2.0 * drand48() - 1.0;
	w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 );

    w = sqrt( (-2.0 * log( w ) ) / w );
    y1 = x1 * w;
    y2 = x2 * w;
    return y1;
}


#define WIDTH 		320
#define HEIGHT		240
#define HALFWIDTH	(WIDTH/2)

typedef float Image[HEIGHT][WIDTH][3] ;
typedef float HalfImage[HEIGHT][HALFWIDTH][3] ;

Image image ;

int
ReadImage(char *fname, Image img) 
{
    FILE *inp ;
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    int stride, x, y, c, idx ;
    JSAMPARRAY buffer ;


    if ((inp = fopen(fname, "rb")) == NULL) {
	fprintf(stderr, "can't open %s\n", fname);
	return 0;
    }

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, inp);
    (void) jpeg_read_header(&cinfo, TRUE);
    (void) jpeg_start_decompress(&cinfo);

    if (cinfo.image_width != WIDTH || 
	cinfo.image_height != HEIGHT) {
	fprintf(stderr, "image size is %dx%d, should be %dx%d.\n",
		cinfo.image_width, cinfo.image_height, WIDTH, HEIGHT) ;
	return 0 ; 
    }

    /* this would be a programming error if it failed. */
    assert(cinfo.output_components == 3) ;

    stride = cinfo.output_width * cinfo.output_components;
    buffer = (*cinfo.mem->alloc_sarray)
		((j_common_ptr) &cinfo, JPOOL_IMAGE, stride, 1);

    while (cinfo.output_scanline < cinfo.output_height) {
	y = cinfo.output_scanline ;
	(void) jpeg_read_scanlines(&cinfo, buffer, 1);
	/* copy out the data into the image */
        for (x=0, idx=0; x<cinfo.output_width; x++) {
	    for(c=0; c<3; c++, idx++)
		img[y][x][c] = buffer[0][idx]/(float)MAXJSAMPLE ;
	}
    }

    (void) jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(inp);
    return 1 ;
}

#define 	SAMPLERATE	11025
#define 	FILESAMPLERATE	11025

SNDFILE *sf ;
SF_INFO sfinfo ;

float complex osc = 0.25 ;

#define BUFFER_SIZE	(256)
float sampbuf[BUFFER_SIZE] ;
int nsamps = 0 ;

/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Bp -o 4 -a 9.9773242630e-02 2.0861678005e-01 -l */

#define NZEROS 8
#define NPOLES 8
#define GAIN   1.555602005e+02

static float xv[NZEROS+1], yv[NPOLES+1];

static float
filtersample(float f)
{
	xv[0] = xv[1]; 
	xv[1] = xv[2]; 
	xv[2] = xv[3]; 
	xv[3] = xv[4]; 
	xv[4] = xv[5]; 
	xv[5] = xv[6]; 
	xv[6] = xv[7]; 
	xv[7] = xv[8]; 
        xv[8] = f / GAIN;
        yv[0] = yv[1]; 
	yv[1] = yv[2]; 
	yv[2] = yv[3]; 
	yv[3] = yv[4]; 
	yv[4] = yv[5]; 
	yv[5] = yv[6]; 
	yv[6] = yv[7]; 
	yv[7] = yv[8]; 
        yv[8] =   (xv[0] + xv[8]) - 4 * (xv[2] + xv[6]) + 6 * xv[4]
                     + ( -0.1604951873 * yv[0]) + (  0.9423465126 * yv[1])
                     + ( -3.0341838933 * yv[2]) + (  6.3246272229 * yv[3])
                     + ( -9.3774455861 * yv[4]) + (  9.9657261890 * yv[5])
                     + ( -7.5658697910 * yv[6]) + (  3.7429505786 * yv[7]);
        return yv[8];
}


void
BufferFloat(float f)
{
    sampbuf[nsamps++] = filtersample(f) ;
    if (nsamps >= BUFFER_SIZE) {
	sf_write_float(sf, sampbuf, BUFFER_SIZE) ;
	nsamps = 0 ;
    }
}

void
BufferFlush()
{
    if (nsamps) 
	sf_write_float(sf, sampbuf, nsamps) ;
    nsamps = 0 ;
}

void
blank(float ms)
{
   int nsamp = (int) (SAMPLERATE * ms / 1000.) ;
   int i ;
   float r = 0.0 ;
   for (i=0; i<nsamp; i++)
	BufferFloat(r) ;
}

void
pulse(float freq, float ms)
{
   /* convert ms to samples */
   int nsamp = (int) (SAMPLERATE * ms / 1000.) ;
   int i ;
   float r ;
   float complex m = cexp(I*2.0*M_PI*freq / SAMPLERATE) ;

   for (i=0; i<nsamp; i++) {
	osc *= m ;
	r = creal(osc) ;
	BufferFloat(r) ;
   }
}

/* The VIS code is 7 bits, transmitted even parity.  My code used
 * to require the user to do that, this was rewritten to fix that.
 */
void
Vis(int code)
{
    int i, p=0 ;

    pulse(1900, 300.0) ;
    pulse(1200, 10.0) ;
    pulse(1900, 300.0) ;
    pulse(1200, 30.0) ;		/* start bit */
    for (i=0; i<7; i++) {
	if (code & 1) {
	    pulse(1100, 30.0) ;
	    p++ ;		/* parity */
	} else {
	    pulse(1300, 30.0) ;
	}
	code = code >> 1 ;
    }

    if (p & 1) {
	pulse(1100, 30.0) ;	/* Output a 1 */
    } else {
	pulse(1300, 30.0) ;	/* Output a 0 */
    }

    pulse(1200, 30.0) ;		/* stop bit */
}

void
Scanline(Image img, HalfImage halfimg, int y)
{
    int i, j ;
    int ysamp = 138 * SAMPLERATE / 1000 ;
    int samp = 69 * SAMPLERATE / 1000 ;

    pulse(1200, 9.0) ;		/* sync pulse */
    pulse(1500, 3.0) ;		/* sync porch */

    /* TRANSMIT THE Y */
    for (i=0; i<ysamp; i++) {
	int idx = i * WIDTH / ysamp ; 
	float f = image[y][idx][0] ;
 	float freq = 1500.0 + (2300.0-1500.0) * f ;
	float complex m = cexp(I*2.0*M_PI*freq/SAMPLERATE) ;
	osc *= m ;
	float r = creal(osc) ;
	BufferFloat(r) ;
    }

    /* TRANSMIT THE R-Y */
    pulse(1500, 4.5) ;		/* even separator pulse */
    pulse(1900, 1.5) ;		/* porch */    
    for (i=0; i<samp; i++) {
	int idx = i * HALFWIDTH / samp ; 
	float f = halfimg[y][idx][1] ;
 	float freq = 1500.0 + (2300.0-1500.0) * f ;
	float complex m = cexp(I*2.0*M_PI*freq/SAMPLERATE) ;
	osc *= m ;
	float r = creal(osc) ;
	BufferFloat(r) ;
    }

    /* TRANSMIT THE B-Y */
    pulse(2300, 4.5) ;		/* sync pulse */
    pulse(1500, 1.5) ;		/* sync porch */
    for (i=0; i<samp; i++) {
	int idx = i * HALFWIDTH / samp ; 
	float f = halfimg[y][idx][2] ;
 	float freq = 1500.0 + (2300.0-1500.0) * f ;
	float complex m = cexp(I*2.0*M_PI*freq/SAMPLERATE) ;
	osc *= m ;
	float r = creal(osc) ;
	BufferFloat(r) ;
    }
}

int
main(int argc, char *argv[])
{
    char *fname = argv[1] ;
    int y, i, j, c ;
    HalfImage halfimage ;

    if (ReadImage(fname, image))
	fprintf(stderr, "%s read successfully.\n", fname) ;

    /* downsample */
    for (j=0; j<HEIGHT; j++) {
	for (i=0; i<HALFWIDTH; i++) {
	    for (c=0; c<3; c++)
		halfimage[j][i][c] = (image[j][2*i][c]+image[j][2*i+1][c]) / 2.;
	}
    }

    /* now, color space convert */
    for (j=0; j<HEIGHT; j++) {
	for (i=0; i<WIDTH; i++) {
	    image[j][i][0] = 0.30 * image[j][i][0] +
			     0.59 * image[j][i][1] +
			     0.11 * image[j][i][2] ;
	}
    }
    /* and the half image */
    for (j=0; j<HEIGHT; j++) {
	for (i=0; i<HALFWIDTH; i++) {
	    float y = 0.30 * halfimage[j][i][0] +
		      0.59 * halfimage[j][i][1] +
		      0.11 * halfimage[j][i][2] ;
	    halfimage[j][i][1] = (halfimage[j][i][1] - y + 1.0) / 2.0 ; 
	    halfimage[j][i][2] = (halfimage[j][i][2] - y + 1.0) / 2.0 ;
	    halfimage[j][i][0] = y ;
	}
     }

    sfinfo.channels = 1 ;
    sfinfo.samplerate = FILESAMPLERATE ;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16 ;

    sf = sf_open(argv[2], SFM_WRITE, &sfinfo) ;

    /* generate the "vis" code. */
    blank(500.0) ;
    Vis(12) ;			/* Robot 72 is VIS mode 12 */

    for (y=0; y<HEIGHT; y ++)
	Scanline(image, halfimage, y) ;

    blank(500.0) ;

    BufferFlush() ;
    sf_close(sf) ;

    return 0 ;
}


/*
 * $Log$
 */

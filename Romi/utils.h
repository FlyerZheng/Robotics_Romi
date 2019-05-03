float rad2deg(float rad)
{
    return rad * (180.0 / PI);
}

float deg2rad(float deg)
{
	  return deg * (PI/ 180.0);
}


/*
 *  This is quite a computationally expensive routine,
 *  so you might want to consider not using it.  But 
 *  gaussian random numbers are really nice for a random
 *  walk behaviour :)
 *  From: http://www.taygeta.com/random/gaussian.html
 */
float randGaussian( float mean, float sd ) {
   float x1, x2, w, y;
   
   do {
     // Adaptation here because arduino random() returns a long
     x1 = random(0,2000) - 1000;
     x1 *= 0.001;
     x2 = random(0,2000) - 1000;
     x2 *= 0.001;
     w = (x1 * x1) + (x2 * x2); 
     
   } while( w >= 1.0 );
   
   w = sqrt( (-2.0 * log( w ) )/w );
   y = x1 * w;
   
   return mean + y * sd;
   
}
 

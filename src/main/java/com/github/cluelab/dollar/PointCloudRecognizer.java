/**
 * The $ gesture recognizers (Java version)
 *
 * Copyright (c) 2018, Mattia De Rosa. All rights reserved.
 *
 * based on the $Q Super-Quick Recognizer (C# version) found at
 * http://depts.washington.edu/madlab/proj/dollar/qdollar.html
 * whose original header follows:
 *
 * The $P Point-Cloud Recognizer (.NET Framework C# version)
 *
 * 	    Radu-Daniel Vatavu, Ph.D.
 *	    University Stefan cel Mare of Suceava
 *	    Suceava 720229, Romania
 *	    vatavu@eed.usv.ro
 *
 *	    Lisa Anthony, Ph.D.
 *      UMBC
 *      Information Systems Department
 *      1000 Hilltop Circle
 *      Baltimore, MD 21250
 *      lanthony@umbc.edu
 *
 *	    Jacob O. Wobbrock, Ph.D.
 * 	    The Information School
 *	    University of Washington
 *	    Seattle, WA 98195-2840
 *	    wobbrock@uw.edu
 *
 * The academic publication for the $P recognizer, and what should be 
 * used to cite it, is:
 *
 *	Vatavu, R.-D., Anthony, L. and Wobbrock, J.O. (2012).  
 *	  Gestures as point clouds: A $P recognizer for user interface 
 *	  prototypes. Proceedings of the ACM Int'l Conference on  
 *	  Multimodal Interfaces (ICMI '12). Santa Monica, California  
 *	  (October 22-26, 2012). New York: ACM Press, pp. 273-280.
 *
 * This software is distributed under the "New BSD License" agreement:
 *
 * Copyright (c) 2012, Radu-Daniel Vatavu, Lisa Anthony, and 
 * Jacob O. Wobbrock. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the names of the University Stefan cel Mare of Suceava, 
 *	    University of Washington, nor UMBC, nor the names of its contributors 
 *	    may be used to endorse or promote products derived from this software 
 *	    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Radu-Daniel Vatavu OR Lisa Anthony
 * OR Jacob O. Wobbrock BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
**/

package com.github.cluelab.dollar;

    /**
     * Implements the $P recognizer
     */
    public class PointCloudRecognizer
    {
        /**
         * Main function of the $P recognizer.
         * Classifies a candidate gesture against a set of training samples.
         * Returns the class of the closest neighbor in the training set.
         *
         * @param candidate
         * @param trainingSet
         * @return
         */
        public static String Classify(Gesture candidate, Gesture[] trainingSet)
        {
            float minDistance = Float.MAX_VALUE;
            String gestureClass = "";
            for (Gesture template : trainingSet)
            {
                float dist = GreedyCloudMatch(candidate.Points, template.Points);
                if (dist < minDistance)
                {
                    minDistance = dist;
                    gestureClass = template.Name;
                }
            }
            return gestureClass;
        }

        /**
         * Compare two gestures.
         */
        public static float GreedyCloudMatch(Gesture gesture1, Gesture gesture2)
        {
            return GreedyCloudMatch(gesture1.Points, gesture2.Points);
        }

        /**
         * Implements greedy search for a minimum-distance matching between two point clouds
         *
         * @param points1
         * @param points2
         * @return
         */
        private static float GreedyCloudMatch(Point[] points1, Point[] points2)
        {
            int n = points1.length; // the two clouds should have the same number of points by now
            float eps = 0.5f;       // controls the number of greedy search trials (eps is in [0..1])
            int step = (int)Math.floor(Math.pow(n, 1.0f - eps));
            float minDistance = Float.MAX_VALUE;
            for (int i = 0; i < n; i += step)
            {
                float dist1 = CloudDistance(points1, points2, i);   // match points1 --> points2 starting with index point i
                float dist2 = CloudDistance(points2, points1, i);   // match points2 --> points1 starting with index point i
                minDistance = Math.min(minDistance, Math.min(dist1, dist2));
            }
            return minDistance;
        }

        /**
         * Computes the distance between two point clouds by performing a minimum-distance greedy matching
         * starting with point startIndex
         *
         * @param points1
         * @param points2
         * @param startIndex
         * @return
         */
        private static float CloudDistance(Point[] points1, Point[] points2, int startIndex)
        {
            int n = points1.length;       // the two clouds should have the same number of points by now
            boolean[] matched = new boolean[n]; // matched[i] signals whether point i from the 2nd cloud has been already matched
            //java.util.Arrays.fill(matched, 0, n, false);   // no points are matched at the beginning

            float sum = 0;  // computes the sum of distances between matched points (i.e., the distance between the two clouds)
            int i = startIndex;
            do
            {
                int index = -1;
                float minDistance = Float.MAX_VALUE;
                for(int j = 0; j < n; j++)
                    if (!matched[j])
                    {
                        float dist = Geometry.EuclideanDistance(points1[i], points2[j]);  
                        if (dist < minDistance)
                        {
                            minDistance = dist;
                            index = j;
                        }
                    }
                matched[index] = true; // point index from the 2nd cloud is matched to point i from the 1st cloud
                float weight = 1.0f - ((i - startIndex + n) % n) / (1.0f * n);
                sum += weight * minDistance; // weight each distance with a confidence coefficient that decreases from 1 to 0
                i = (i + 1) % n;
            } while (i != startIndex);
            return sum;
        }
    }

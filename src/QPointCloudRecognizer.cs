/**
 * The $Q Point-Cloud Recognizer (.NET Framework C# version)
 *
 * 	    Radu-Daniel Vatavu, Ph.D.
 *	    University Stefan cel Mare of Suceava
 *	    Suceava 720229, Romania
 *	    radu.vatavu@usm.ro
 *
 *	    Lisa Anthony, Ph.D.
 *      Department of CISE
 *      University of Florida
 *      Gainesville, FL 32611, USA
 *      lanthony@cise.ufl.edu
 *
 *	    Jacob O. Wobbrock, Ph.D.
 * 	    The Information School
 *	    University of Washington
 *	    Seattle, WA 98195-2840
 *	    wobbrock@uw.edu
 *
 * The academic publication for the $Q recognizer, and what should be 
 * used to cite it, is:
 *
 *	Vatavu, R.-D., Anthony, L. and Wobbrock, J.O. (2018).  
 *	  $Q: A Super-Quick, Articulation-Invariant Stroke-Gesture
 *    Recognizer for Low-Resource Devices. Proceedings of 20th International Conference on
 *    Human-Computer Interaction with Mobile Devices and Services (MobileHCI '18). Barcelona, Spain
 *	  (September 3-6, 2018). New York: ACM Press.
 *	  DOI: https://doi.org/10.1145/3229434.3229465
 *
 * This software is distributed under the "New BSD License" agreement:
 *
 * Copyright (c) 2018, Radu-Daniel Vatavu, Lisa Anthony, and 
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
 *	    University of Washington, nor University of Florida, nor the names of its contributors 
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
using System;
using PDollarGestureRecognizer;

namespace QDollarGestureRecognizer
{
    /// <summary>
    /// Implements the $Q recognizer
    /// </summary>
    public class QPointCloudRecognizer
    {
        // $Q's two major optimization layers (Early Abandoning and Lower Bounding)
        // can be activated / deactivated as desired
        public static bool UseEarlyAbandoning = true;
        public static bool UseLowerBounding = true;

        /// <summary>
        /// Main function of the $Q recognizer.
        /// Classifies a candidate gesture against a set of templates.
        /// Returns the class of the closest neighbor in the template set.
        /// </summary>
        /// <param name="candidate"></param>
        /// <param name="templateSet"></param>
        /// <returns></returns>
        public static string Classify(Gesture candidate, Gesture[] templateSet)
        {
            float minDistance = float.MaxValue;
            string gestureClass = "";
            foreach (Gesture template in templateSet)
            {
                float dist = GreedyCloudMatch(candidate, template, minDistance);
                if (dist < minDistance)
                {
                    minDistance = dist;
                    gestureClass = template.Name;
                }
            }
            return gestureClass;
        }

        /// <summary>
        /// Implements greedy search for a minimum-distance matching between two point clouds.
        /// Implements Early Abandoning and Lower Bounding (LUT) optimizations.
        /// </summary>
        private static float GreedyCloudMatch(Gesture gesture1, Gesture gesture2, float minSoFar)
        {
            int n = gesture1.Points.Length;       // the two clouds should have the same number of points by now
            float eps = 0.5f;                     // controls the number of greedy search trials (eps is in [0..1])
            int step = (int)Math.Floor(Math.Pow(n, 1.0f - eps));

            if (UseLowerBounding)
            {
                float[] LB1 = ComputeLowerBound(gesture1.Points, gesture2.Points, gesture2.LUT, step);  // direction of matching: gesture1 --> gesture2
                float[] LB2 = ComputeLowerBound(gesture2.Points, gesture1.Points, gesture1.LUT, step);  // direction of matching: gesture2 --> gesture1
                for (int i = 0, indexLB = 0; i < n; i += step, indexLB++)
                {
                    if (LB1[indexLB] < minSoFar) minSoFar = Math.Min(minSoFar, CloudDistance(gesture1.Points, gesture2.Points, i, minSoFar));  // direction of matching: gesture1 --> gesture2 starting with index point i
                    if (LB2[indexLB] < minSoFar) minSoFar = Math.Min(minSoFar, CloudDistance(gesture2.Points, gesture1.Points, i, minSoFar));  // direction of matching: gesture2 --> gesture1 starting with index point i   
                }
            }
            else
            {
                for (int i = 0; i < n; i += step)
                {
                    minSoFar = Math.Min(minSoFar, CloudDistance(gesture1.Points, gesture2.Points, i, minSoFar));  // direction of matching: gesture1 --> gesture2 starting with index point i
                    minSoFar = Math.Min(minSoFar, CloudDistance(gesture2.Points, gesture1.Points, i, minSoFar));  // direction of matching: gesture2 --> gesture1 starting with index point i   
                }
            }

            return minSoFar;
        }

        /// <summary>
        /// Computes lower bounds for each starting point and the direction of matching from points1 to points2 
        /// </summary>
        private static float[] ComputeLowerBound(Point[] points1, Point[] points2, int[][] LUT, int step)
        {
            int n = points1.Length;
            float[] LB = new float[n / step + 1];
            float[] SAT = new float[n];

            LB[0] = 0;
            for (int i = 0; i < n; i++)
            {
                int index = LUT[points1[i].intY / Gesture.LUT_SCALE_FACTOR][points1[i].intX / Gesture.LUT_SCALE_FACTOR];
                float dist = Geometry.SqrEuclideanDistance(points1[i], points2[index]);
                SAT[i] = (i == 0) ? dist : SAT[i - 1] + dist;
                LB[0] += (n - i) * dist;
            }

            for (int i = step, indexLB = 1; i < n; i += step, indexLB++)
                LB[indexLB] = LB[0] + i * SAT[n - 1] - n * SAT[i - 1];
            return LB;
        }

        /// <summary>
        /// Computes the distance between two point clouds by performing a minimum-distance greedy matching
        /// starting with point startIndex
        /// </summary>
        /// <param name="points1"></param>
        /// <param name="points2"></param>
        /// <param name="startIndex"></param>
        /// <returns></returns>
        private static float CloudDistance(Point[] points1, Point[] points2, int startIndex, float minSoFar)
        {
            int n = points1.Length;                // the two point clouds should have the same number of points by now
            int[] indexesNotMatched = new int[n];  // stores point indexes for points from the 2nd cloud that haven't been matched yet
            for (int j = 0; j < n; j++)
                indexesNotMatched[j] = j;

            float sum = 0;                // computes the sum of distances between matched points (i.e., the distance between the two clouds)
            int i = startIndex;           // start matching with point startIndex from the 1st cloud
            int weight = n;               // implements weights, decreasing from n to 1
            int indexNotMatched = 0;      // indexes the indexesNotMatched[..] array of points from the 2nd cloud that are not matched yet
            do
            {
                int index = -1;
                float minDistance = float.MaxValue;
                for (int j = indexNotMatched; j < n; j++)
                {
                    float dist = Geometry.SqrEuclideanDistance(points1[i], points2[indexesNotMatched[j]]);  // use the squared Euclidean distance
                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        index = j;
                    }
                }
                indexesNotMatched[index] = indexesNotMatched[indexNotMatched];  // point indexesNotMatched[index] of the 2nd cloud is now matched to point i of the 1st cloud
                sum += (weight--) * minDistance;           // weight each distance with a confidence coefficient that decreases from n to 1

                if (UseEarlyAbandoning)
                {
                    if (sum >= minSoFar) 
                        return sum;       // implement early abandoning
                }

                i = (i + 1) % n;                           // advance to the next point in the 1st cloud
                indexNotMatched++;                         // update the number of points from the 2nd cloud that haven't been matched yet
            } while (i != startIndex);
            return sum;
        }
    }
}
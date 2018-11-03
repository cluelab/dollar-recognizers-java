/**
 * The $P+ Point-Cloud Recognizer (.NET Framework C# version)
 *
 * 	    Radu-Daniel Vatavu, Ph.D.
 *	    University Stefan cel Mare of Suceava
 *	    Suceava 720229, Romania
 *	    radu.vatavu@usm.ro
 *
 * The academic publication for the $P+ recognizer, and what should be 
 * used to cite it, is:
 *
 *	Vatavu, R.-D. (2017). Improving Gesture Recognition Accuracy on Touch Screens for Users with Low Vision.
 *  In Proceedings of CHI '17, the 35th ACM Conference on Human Factors in Computing Systems (Denver, Colorado, USA, May 2017). 
 *  New York: ACM Press. http://dx.doi.org/10.1145/3025453.3025941
 *
 * This software is distributed under the "New BSD License" agreement:
 *
 * Copyright (c) 2017, Radu-Daniel Vatavu. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the University Stefan cel Mare of Suceava, 
 *	    nor the names of its contributors may be used to endorse or promote products 
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Radu-Daniel Vatavu BE LIABLE FOR 
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
using System;

namespace PDollarGestureRecognizer
{
    /// <summary>
    /// Implements the $P+ recognizer
    /// </summary>
    public class PointCloudRecognizerPlus
    {
        /// <summary>
        /// Main function of the $P+ recognizer.
        /// Classifies a candidate gesture against a set of training samples.
        /// Returns the class of the closest neighbor in the template set.
        /// </summary>
        /// <param name="candidate"></param>
        /// <param name="trainingSet"></param>
        /// <returns></returns>
        public static string Classify(Gesture candidate, Gesture[] trainingSet)
        {
            float minDistance = float.MaxValue;
            string gestureClass = "";
            foreach (Gesture template in trainingSet)
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

        /// <summary>
        /// Implements greedy search for a minimum-distance matching between two point clouds
        /// using local shape descriptors (theta turning angles).
        /// </summary>
        private static float GreedyCloudMatch(Point[] points1, Point[] points2)
        {
            float[] theta1 = ComputeLocalShapeDescriptors(points1);    // should be pre-processed in the Gesture class
            float[] theta2 = ComputeLocalShapeDescriptors(points2);    // should be pre-processed in the Gesture class
            return Math.Min(
                CloudDistance(points1, theta1, points2, theta2), 
                CloudDistance(points2, theta2, points1, theta1)
            );
        }

        /// <summary>
        /// Computes the distance between two point clouds 
        /// using local shape descriptors (theta turning angles).
        /// </summary>
        private static float CloudDistance(Point[] points1, float[] theta1, Point[] points2, float[] theta2)
        {
            bool[] matched = new bool[points2.Length];
            Array.Clear(matched, 0, points2.Length);

            float sum = 0; // computes the cost of the cloud alignment
            int index;

            // match points1 to points2
            for (int i = 0; i < points1.Length; i++)
            {
                sum += GetClosestPointFromCloud(points1[i], theta1[i], points2, theta2, out index);
                matched[index] = true;
            }

            // match points2 to points1
            for (int i = 0; i < points2.Length; i++)
                if (!matched[i])
                    sum += GetClosestPointFromCloud(points2[i], theta2[i], points1, theta1, out index);

            return sum;
        }


        /// <summary>
        /// Searches for the point from point-cloud cloud that is closest to point p.
        /// </summary>
        private static float GetClosestPointFromCloud(Point p, float theta, Point[] cloud, float[] thetaCloud, out int indexMin)
        {
            float min = float.MaxValue;
            indexMin = -1;
            for (int i = 0; i < cloud.Length; i++)
            {
                float dist = (float)Math.Sqrt(Geometry.SqrEuclideanDistance(p, cloud[i]) + (theta - thetaCloud[i]) * (theta - thetaCloud[i]));
                if (dist < min)
                {
                    min = dist;
                    indexMin = i;
                }
            }
            return min;
        }

        /// <summary>
        /// Computes local shape descriptors (theta turning angles) at each point on the gesture path.
        /// </summary>
        public static float[] ComputeLocalShapeDescriptors(Point[] points)
        {
            int n = points.Length;
            float[] theta = new float[n];

            theta[0] = theta[n - 1] = 0;
            for (int i = 1; i < n - 1; i++)
                theta[i] = (float)(ShortAngle(points[i - 1], points[i], points[i + 1]) / Math.PI);
            return theta;
        }

        /// <summary>
        /// Computes the smallest turning angle between vectors (a,b) and (b,c) in radians in the interval [0..PI].
		/// </summary>
        public static float ShortAngle(Point a, Point b, Point c)
        {
            // compute path lengths for vectors (a,b) and (b,c)
            float length_ab = Geometry.EuclideanDistance(a, b);
            float length_bc = Geometry.EuclideanDistance(b, c);
            if (Math.Abs(length_ab * length_bc) <= float.Epsilon)
                return 0;

            // compute cosine of the angle between vectors (a,b) and (b,c)
            double cos_angle = ((b.X - a.X) * (c.X - b.X) + (b.Y - a.Y) * (c.Y - b.Y)) / (length_ab * length_bc);

            // deal with special cases near limits of the [-1,1] interval
            if (cos_angle <= -1.0) return (float)Math.PI;
            if (cos_angle >= 1.0) return 0.0f;

            // return the angle between vectors (a,b) and (b,c) in the interval [0,PI]
            return (float)Math.Acos(cos_angle);
        }
    }
}
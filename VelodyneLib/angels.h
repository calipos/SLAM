#ifndef _SELF_ROS_H_
#define _SELF_ROS_H_
#include <iostream>
#include <algorithm>
#include <math.h>
#define PIx2 (6.28318530717958647692528676656)
#define PI (3.1415926535897932384626433832795)
namespace angles
{

    /*!
     * \brief Convert degrees to radians
     */

    static inline double from_degrees(double degrees)
    {
        return degrees * PI / 180.0;
    }

    /*!
     * \brief Convert radians to degrees
     */
    static inline double to_degrees(double radians)
    {
        return radians * 180.0 / PI;
    }


    /*!
     * \brief normalize_angle_positive
     *
     *        Normalizes the angle to be 0 to PIx2
     *        It takes and returns radians.
     */
    static inline double normalize_angle_positive(double angle)
    {
        return fmod(fmod(angle, PIx2) + PIx2, PIx2);
    }


    /*!
     * \brief normalize
     *
     * Normalizes the angle to be -PI circle to +PI circle
     * It takes and returns radians.
     *
     */
    static inline double normalize_angle(double angle)
    {
        double a = normalize_angle_positive(angle);
        if (a > PI)
            a -= PIx2;
        return a;
    }


    /*!
     * \function
     * \brief shortest_angular_distance
     *
     * Given 2 angles, this returns the shortest angular
     * difference.  The inputs and ouputs are of course radians.
     *
     * The result
     * would always be -pi <= result <= pi.  Adding the result
     * to "from" will always get you an equivelent angle to "to".
     */

    static inline double shortest_angular_distance(double from, double to)
    {
        return normalize_angle(to - from);
    }

    /*!
     * \function
     *
     * \brief returns the angle in [-PIx2, PIx2]  going the other way along the unit circle.
     * \param angle The angle to which you want to turn in the range [-PIx2, PIx2]
     * E.g. two_pi_complement(-PI/4) returns 7_M_PI/4
     * two_pi_complement(PI/4) returns -7*PI/4
     *
     */
    static inline double two_pi_complement(double angle)
    {
        //check input conditions
        if (angle > PIx2 || angle < -PIx2)
            angle = fmod(angle, PIx2);
        if (angle < 0)
            return (PIx2 + angle);
        else if (angle > 0)
            return (-PIx2 + angle);

        return(PIx2);
    }

    /*!
     * \function
     *
     * \brief This function is only intended for internal use and not intended for external use. If you do use it, read the documentation very carefully. Returns the min and max amount (in radians) that can be moved from "from" angle to "left_limit" and "right_limit".
     * \return returns false if "from" angle does not lie in the interval [left_limit,right_limit]
     * \param from - "from" angle - must lie in [-PI, PI)
     * \param left_limit - left limit of valid interval for angular position - must lie in [-PI, PI], left and right limits are specified on the unit circle w.r.t to a reference pointing inwards
     * \param right_limit - right limit of valid interval for angular position - must lie in [-PI, PI], left and right limits are specified on the unit circle w.r.t to a reference pointing inwards
     * \param result_min_delta - minimum (delta) angle (in radians) that can be moved from "from" position before hitting the joint stop
     * \param result_max_delta - maximum (delta) angle (in radians) that can be movedd from "from" position before hitting the joint stop
     */
    static bool find_min_max_delta(double from, double left_limit, double right_limit, double& result_min_delta, double& result_max_delta)
    {
        double delta[4];

        delta[0] = shortest_angular_distance(from, left_limit);
        delta[1] = shortest_angular_distance(from, right_limit);

        delta[2] = two_pi_complement(delta[0]);
        delta[3] = two_pi_complement(delta[1]);

        if (delta[0] == 0)
        {
            result_min_delta = delta[0];
            result_max_delta = std::max<double>(delta[1], delta[3]);
            return true;
        }

        if (delta[1] == 0)
        {
            result_max_delta = delta[1];
            result_min_delta = std::min<double>(delta[0], delta[2]);
            return true;
        }


        double delta_min = delta[0];
        double delta_min_2pi = delta[2];
        if (delta[2] < delta_min)
        {
            delta_min = delta[2];
            delta_min_2pi = delta[0];
        }

        double delta_max = delta[1];
        double delta_max_2pi = delta[3];
        if (delta[3] > delta_max)
        {
            delta_max = delta[3];
            delta_max_2pi = delta[1];
        }


        //    printf("%f %f %f %f\n",delta_min,delta_min_2pi,delta_max,delta_max_2pi);
        if ((delta_min <= delta_max_2pi) || (delta_max >= delta_min_2pi))
        {
            result_min_delta = delta_max_2pi;
            result_max_delta = delta_min_2pi;
            if (left_limit == -PI && right_limit == PI)
                return true;
            else
                return false;
        }
        result_min_delta = delta_min;
        result_max_delta = delta_max;
        return true;
    }


    /*!
     * \function
     *
     * \brief Returns the delta from `from_angle` to `to_angle`, making sure it does not violate limits specified by `left_limit` and `right_limit`.
     * This function is similar to `shortest_angular_distance_with_limits()`, with the main difference that it accepts limits outside the `[-PI, PI]` range.
     * Even if this is quite uncommon, one could indeed consider revolute joints with large rotation limits, e.g., in the range `[-PIx2, PIx2]`.
     *
     * In this case, a strict requirement is to have `left_limit` smaller than `right_limit`.
     * Note also that `from` must lie inside the valid range, while `to` does not need to.
     * In fact, this function will evaluate the shortest (valid) angle `shortest_angle` so that `from+shortest_angle` equals `to` up to an integer multiple of `PIx2`.
     * As an example, a call to `shortest_angular_distance_with_large_limits(0, 10.5*PI, -PIx2, PIx2, shortest_angle)` will return `true`, with `shortest_angle=0.5*PI`.
     * This is because `from` and `from+shortest_angle` are both inside the limits, and `fmod(to+shortest_angle, PIx2)` equals `fmod(to, PIx2)`.
     * On the other hand, `shortest_angular_distance_with_large_limits(10.5*PI, 0, -PIx2, PIx2, shortest_angle)` will return false, since `from` is not in the valid range.
     * Finally, note that the call `shortest_angular_distance_with_large_limits(0, 10.5*PI, -PIx2, 0.1*PI, shortest_angle)` will also return `true`.
     * However, `shortest_angle` in this case will be `-1.5*PI`.
     *
     * \return true if `left_limit < right_limit` and if "from" and "from+shortest_angle" positions are within the valid interval, false otherwise.
     * \param from - "from" angle.
     * \param to - "to" angle.
     * \param left_limit - left limit of valid interval, must be smaller than right_limit.
     * \param right_limit - right limit of valid interval, must be greater than left_limit.
     * \param shortest_angle - result of the shortest angle calculation.
     */
    static inline bool shortest_angular_distance_with_large_limits(double from, double to, double left_limit, double right_limit, double& shortest_angle)
    {
        // Shortest steps in the two directions
        double delta = shortest_angular_distance(from, to);
        double delta_2pi = two_pi_complement(delta);

        // "sort" distances so that delta is shorter than delta_2pi
        if (std::fabs(delta) > std::fabs(delta_2pi))
            std::swap(delta, delta_2pi);

        if (left_limit > right_limit) {
            // If limits are something like [PI/2 , -PI/2] it actually means that we
            // want rotations to be in the interval [-PI,PI/2] U [PI/2,PI], ie, the
            // half unit circle not containing the 0. This is already gracefully
            // handled by shortest_angular_distance_with_limits, and therefore this
            // function should not be called at all. However, if one has limits that
            // are larger than PI, the same rationale behind shortest_angular_distance_with_limits
            // does not hold, ie, PI+x should not be directly equal to -PI+x.
            // In this case, the correct way of getting the shortest solution is to
            // properly set the limits, eg, by saying that the interval is either
            // [PI/2, 3*PI/2] or [-3*PI/2, -PI/2]. For this reason, here we
            // return false by default.
            shortest_angle = delta;
            return false;
        }

        // Check in which direction we should turn (clockwise or counter-clockwise).

        // start by trying with the shortest angle (delta).
        double to2 = from + delta;
        if (left_limit <= to2 && to2 <= right_limit) {
            // we can move in this direction: return success if the "from" angle is inside limits
            shortest_angle = delta;
            return left_limit <= from && from <= right_limit;
        }

        // delta is not ok, try to move in the other direction (using its complement)
        to2 = from + delta_2pi;
        if (left_limit <= to2 && to2 <= right_limit) {
            // we can move in this direction: return success if the "from" angle is inside limits
            shortest_angle = delta_2pi;
            return left_limit <= from && from <= right_limit;
        }

        // nothing works: we always go outside limits
        shortest_angle = delta; // at least give some "coherent" result
        return false;
    }


    /*!
     * \function
     *
     * \brief Returns the delta from "from_angle" to "to_angle" making sure it does not violate limits specified by left_limit and right_limit.
     * The valid interval of angular positions is [left_limit,right_limit]. E.g., [-0.25,0.25] is a 0.5 radians wide interval that contains 0.
     * But [0.25,-0.25] is a PIx2-0.5 wide interval that contains PI (but not 0).
     * The value of shortest_angle is the angular difference between "from" and "to" that lies within the defined valid interval.
     * E.g. shortest_angular_distance_with_limits(-0.5,0.5,0.25,-0.25,ss) evaluates ss to PIx2-1.0 and returns true while
     * shortest_angular_distance_with_limits(-0.5,0.5,-0.25,0.25,ss) returns false since -0.5 and 0.5 do not lie in the interval [-0.25,0.25]
     *
     * \return true if "from" and "to" positions are within the limit interval, false otherwise
     * \param from - "from" angle
     * \param to - "to" angle
     * \param left_limit - left limit of valid interval for angular position, left and right limits are specified on the unit circle w.r.t to a reference pointing inwards
     * \param right_limit - right limit of valid interval for angular position, left and right limits are specified on the unit circle w.r.t to a reference pointing inwards
     * \param shortest_angle - result of the shortest angle calculation
     */
    static inline bool shortest_angular_distance_with_limits(double from, double to, double left_limit, double right_limit, double& shortest_angle)
    {

        double min_delta = -PIx2;
        double max_delta = PIx2;
        double min_delta_to = -PIx2;
        double max_delta_to = PIx2;
        bool flag = find_min_max_delta(from, left_limit, right_limit, min_delta, max_delta);
        double delta = shortest_angular_distance(from, to);
        double delta_mod_2pi = two_pi_complement(delta);


        if (flag)//from position is within the limits
        {
            if (delta >= min_delta && delta <= max_delta)
            {
                shortest_angle = delta;
                return true;
            }
            else if (delta_mod_2pi >= min_delta && delta_mod_2pi <= max_delta)
            {
                shortest_angle = delta_mod_2pi;
                return true;
            }
            else //to position is outside the limits
            {
                find_min_max_delta(to, left_limit, right_limit, min_delta_to, max_delta_to);
                if (fabs(min_delta_to) < fabs(max_delta_to))
                    shortest_angle = std::max<double>(delta, delta_mod_2pi);
                else if (fabs(min_delta_to) > fabs(max_delta_to))
                    shortest_angle = std::min<double>(delta, delta_mod_2pi);
                else
                {
                    if (fabs(delta) < fabs(delta_mod_2pi))
                        shortest_angle = delta;
                    else
                        shortest_angle = delta_mod_2pi;
                }
                return false;
            }
        }
        else // from position is outside the limits
        {
            find_min_max_delta(to, left_limit, right_limit, min_delta_to, max_delta_to);

            if (fabs(min_delta) < fabs(max_delta))
                shortest_angle = std::min<double>(delta, delta_mod_2pi);
            else if (fabs(min_delta) > fabs(max_delta))
                shortest_angle = std::max<double>(delta, delta_mod_2pi);
            else
            {
                if (fabs(delta) < fabs(delta_mod_2pi))
                    shortest_angle = delta;
                else
                    shortest_angle = delta_mod_2pi;
            }
            return false;
        }

        shortest_angle = delta;
        return false;
    }
}

#endif // !_SELF_ROS_H_

#include "datmo.hpp"

// DETECT MOTION FOR LASER
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::store_background()
{
    // store all the hits of the laser in the background table
    // make sure background containers have the right size
    background_ranges_.resize(nb_beams_);
    background_points_.resize(nb_beams_);

    for (int loop_hit = 0; loop_hit < nb_beams_; ++loop_hit)
    {
        // copy current range and point into the background
        background_ranges_[loop_hit] = ranges_[loop_hit];
        background_points_[loop_hit] = points_[loop_hit];
    }
} // store_background

void Datmo::reset_motion()
{
    // reset the dynamic table for each laser hit
    dynamic_.resize(nb_beams_);
    for (int loop_hit = 0; loop_hit < nb_beams_; ++loop_hit)
        dynamic_[loop_hit] = false;
} // reset_motion

void Datmo::detect_current_motion()
{
    // for each hit, compare the current range/point with the background to detect motion
    // fill the table dynamic_. Use a small threshold (meters) to decide motion.
    const float range_motion_threshold = 0.05f; // 5 cm
    const float point_motion_threshold = 0.05f; // 5 cm fallback using Euclidean distance

    // ensure dynamic_ has the right size
    dynamic_.resize(nb_beams_);

    for (int loop_hit = 0; loop_hit < nb_beams_; ++loop_hit)
    {
        dynamic_[loop_hit] = false;

        // basic checks
        if (loop_hit >= static_cast<int>(background_ranges_.size()) ||
            loop_hit >= static_cast<int>(background_points_.size()))
            continue;

        float br = background_ranges_[loop_hit];
        float cr = ranges_[loop_hit];

        // ignore invalid ranges (<= 0)
        if (!(br > 0.0f) || !(cr > 0.0f))
            continue;

        // compare ranges first
        if (std::fabs(cr - br) > range_motion_threshold)
        {
            dynamic_[loop_hit] = true;
            continue;
        }

        // fallback: compare 2D/3D points if ranges are similar
        if (loop_hit < static_cast<int>(points_.size()) &&
            loop_hit < static_cast<int>(background_points_.size()))
        {
            if (this->distancePoints(points_[loop_hit], background_points_[loop_hit]) > point_motion_threshold)
                dynamic_[loop_hit] = true;
        }
    }
} // detect_current_motion

void Datmo::detect_motion()
{
    // integrate store_background, reset_motion and detect_current_motion
    if (!current_robot_moving_)
    {
        // robot is not moving -> we can perform moving person detection
        if (!previous_robot_moving_)
        {
            // robot was not moving and still isn't -> detect motion comparing current scan to stored background
            detect_current_motion();
        }
        else
        {
            // robot was moving and just stopped -> store the current scan as background and reset dynamic table
            store_background();
            reset_motion();
        }
    }
    else
    {
        // robot is moving -> impossible to detect motions reliably, mark all hits as non-dynamic
        reset_motion();

        // if the robot just started moving, invalidate stored background to avoid wrong comparisons later
        if (!previous_robot_moving_)
        {
            background_ranges_.clear();
            background_points_.clear();
        }
    }

    // update previous state for next iteration
    previous_robot_moving_ = current_robot_moving_;
} // detect_motion

// CLUSTERING FOR LASER DATA
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::perform_clustering(float clustering_threshold)
{
    perform_basic_clustering(clustering_threshold);
    perform_advanced_clustering();
} // perform_clustering

void Datmo::perform_basic_clustering(float cluster_threshold)
{
    // clustering per lectures: data stored in cluster_start_, cluster_end_, nb_clusters_
    nb_clusters_ = 0;
    cluster_start_.clear();
    cluster_end_.clear();

    if (nb_beams_ <= 0)
        return;

    // start first cluster at hit 0
    cluster_start_.push_back(0);

    for (int loop_hit = 1; loop_hit < nb_beams_; ++loop_hit)
    {
        // validity checks for ranges: if either hit is invalid, treat as boundary
        bool prev_valid = (loop_hit - 1 < static_cast<int>(ranges_.size())) && (ranges_[loop_hit - 1] > 0.0f);
        bool cur_valid  = (loop_hit     < static_cast<int>(ranges_.size())) && (ranges_[loop_hit]     > 0.0f);

        bool boundary = false;
        if (!prev_valid || !cur_valid)
        {
            boundary = true;
        }
        else
        {
            // distance between consecutive points
            if (loop_hit < static_cast<int>(points_.size()) && (loop_hit - 1) < static_cast<int>(points_.size()))
            {
                if (this->distancePoints(points_[loop_hit - 1], points_[loop_hit]) > cluster_threshold)
                    boundary = true;
            }
            else
            {
                boundary = true;
            }
        }

        if (boundary)
        {
            // end previous cluster at loop_hit-1 and start a new one at loop_hit
            cluster_end_.push_back(loop_hit - 1);
            cluster_start_.push_back(loop_hit);
        }
    }

    // close last cluster
    cluster_end_.push_back(nb_beams_ - 1);

    // number of clusters
    nb_clusters_ = static_cast<int>(cluster_start_.size());
}

void Datmo::perform_advanced_clustering()
{
    // for each cluster, update derived info
    cluster_size_.resize(nb_clusters_);
    cluster_middle_.resize(nb_clusters_);
    cluster_dynamic_.resize(nb_clusters_);
    cluster_nb_points_.resize(nb_clusters_);

    for (int loop_cluster = 0; loop_cluster < nb_clusters_; loop_cluster++)
    {
        int start = cluster_start_[loop_cluster];
        int end   = cluster_end_[loop_cluster];

        // defaults
        cluster_size_[loop_cluster]     = 0.0f;
        cluster_middle_[loop_cluster].x = 0.0;
        cluster_middle_[loop_cluster].y = 0.0;
        cluster_middle_[loop_cluster].z = 0.0;
        cluster_dynamic_[loop_cluster]  = 0.0f;
        cluster_nb_points_[loop_cluster]= 0;

        if (start > end || start < 0 || end < 0 || nb_beams_ <= 0)
            continue;

        int nb_points = end - start + 1;
        cluster_nb_points_[loop_cluster] = nb_points;

        // compute cluster size and middle if points are available
        if (start < static_cast<int>(points_.size()) && end < static_cast<int>(points_.size()))
        {
            cluster_size_[loop_cluster] = this->distancePoints(points_[start], points_[end]);
            // midpoint between first and last hit
            cluster_middle_[loop_cluster].x = 0.5 * (points_[start].x + points_[end].x);
            cluster_middle_[loop_cluster].y = 0.5 * (points_[start].y + points_[end].y);
            cluster_middle_[loop_cluster].z = 0.5 * (points_[start].z + points_[end].z);
        }
        else if (start < static_cast<int>(points_.size()))
        {
            cluster_middle_[loop_cluster] = points_[start];
            cluster_size_[loop_cluster] = 0.0f;
        }
        else if (end < static_cast<int>(points_.size()))
        {
            cluster_middle_[loop_cluster] = points_[end];
            cluster_size_[loop_cluster] = 0.0f;
        }

        // % of dynamic hits in the cluster
        int dynamic_count = 0;
        for (int idx = start; idx <= end; ++idx)
        {
            if (idx >= 0 && idx < static_cast<int>(dynamic_.size()) && dynamic_[idx])
                ++dynamic_count;
        }

        if (nb_points > 0)
            cluster_dynamic_[loop_cluster] = 100.0f * static_cast<float>(dynamic_count) / static_cast<float>(nb_points);
        else
            cluster_dynamic_[loop_cluster] = 0.0f;
    }
} // perform_advanced_clustering

int Datmo::compute_nb_dynamic(int start, int end)
{
    // return the number of points that are dynamic between start and end (inclusive)
    int nb_dynamic = 0;

    if (start > end)
        return 0;

    for (int i = start; i <= end; ++i)
    {
        if (i >= 0 && i < static_cast<int>(dynamic_.size()) && dynamic_[i])
            ++nb_dynamic;
    }
    return nb_dynamic;
} // compute_nb_dynamic

// DETECTION OF PERSONS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::detect_legs()
{
    // a leg is a cluster:
    //   - with a size higher than "leg_size_min_";
    //   - with a size lower than "leg_size_max_";
    //   - with more than 5 hits
    // if more than "dynamic_threshold_"% of its hits are dynamic the leg is considered to be dynamic
    // we update the arrays leg_cluster_, leg_detected_ and leg_dynamic_
    nb_legs_detected_ = 0;
    leg_cluster_.clear();
    leg_detected_.clear();
    leg_dynamic_.clear();

    for (int loop = 0; loop < nb_clusters_; ++loop) // loop over all the clusters
    {
        int nb_points = 0;
        if (loop < static_cast<int>(cluster_nb_points_.size()))
            nb_points = cluster_nb_points_[loop];

        float size = 0.0f;
        if (loop < static_cast<int>(cluster_size_.size()))
            size = cluster_size_[loop];

        float dyn_percent = 0.0f;
        if (loop < static_cast<int>(cluster_dynamic_.size()))
            dyn_percent = cluster_dynamic_[loop];

        bool is_leg        = (nb_points > 5) && (size >= leg_size_min_) && (size <= leg_size_max_);
        bool is_dynamic_leg= is_leg && (dyn_percent >= dynamic_threshold_);

        if (is_leg)
        {
            leg_cluster_.push_back(loop);
            leg_detected_.push_back(true);
            leg_dynamic_.push_back(is_dynamic_leg);
            ++nb_legs_detected_;
        }
    }
} // detect_legs

void Datmo::detect_persons()
{
    // a person has two legs located at less than "legs_distance_max_" one from the other
    // a moving person (ie, person_dynamic_ array) has 2 legs that are dynamic
    // we update the detected_person_ table to store the middle of the person
    // we update the person_dynamic_ table to know if the person is moving or not

    nb_persons_detected_ = 0;
    detected_person_.clear();
    person_dynamic_.clear();

    // safety checks
    if (nb_legs_detected_ <= 0)
        return;

    for (int loop_leg_left = 0; loop_leg_left < nb_legs_detected_; ++loop_leg_left)
    {
        for (int loop_leg_right = loop_leg_left + 1; loop_leg_right < nb_legs_detected_; ++loop_leg_right)
        {
            // get cluster indices for the two legs
            int cluster_left  = leg_cluster_[loop_leg_left];
            int cluster_right = leg_cluster_[loop_leg_right];

            // bounds check for cluster_middle_
            if (cluster_left < 0 || cluster_left >= static_cast<int>(cluster_middle_.size()) ||
                cluster_right < 0 || cluster_right >= static_cast<int>(cluster_middle_.size()))
                continue;

            // distance between the two leg midpoints
            float dist = this->distancePoints(cluster_middle_[cluster_left], cluster_middle_[cluster_right]);

            // if legs are close enough, consider them a person
            if (dist <= legs_distance_max_)
            {
                // person middle is midpoint between the two legs
                geometry_msgs::msg::Point person_mid;
                person_mid.x = 0.5f * (cluster_middle_[cluster_left].x + cluster_middle_[cluster_right].x);
                person_mid.y = 0.5f * (cluster_middle_[cluster_left].y + cluster_middle_[cluster_right].y);
                person_mid.z = 0.5f * (cluster_middle_[cluster_left].z + cluster_middle_[cluster_right].z);

                detected_person_.push_back(person_mid);

                // person is dynamic if both legs are dynamic
                bool is_dyn = false;
                if (loop_leg_left < static_cast<int>(leg_dynamic_.size()) &&
                    loop_leg_right < static_cast<int>(leg_dynamic_.size()))
                {
                    is_dyn = leg_dynamic_[loop_leg_left] && leg_dynamic_[loop_leg_right];
                }
                person_dynamic_.push_back(is_dyn);

                ++nb_persons_detected_;
            }
        }
    }
} // detect_persons

void Datmo::detect_a_moving_person()
{
    // store the first moving detected person into moving_detected_person_
    // and set is_moving_person_detected_
    is_moving_person_detected_ = false;
    for (int loop_persons = 0; loop_persons < nb_persons_detected_; loop_persons++)
    {
        if (loop_persons < static_cast<int>(person_dynamic_.size()) && person_dynamic_[loop_persons])
        {
            if (loop_persons < static_cast<int>(detected_person_.size()))
            {
                moving_detected_person_ = detected_person_[loop_persons];
            }
            is_moving_person_detected_ = true;
            break; // take the first moving person found
        }
    }
    /*if ( is_moving_person_detected_ )
        pub_detection_.publish(moving_detected_person_);*/
} // detect_a_moving_person

// TRACKING OF A PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void Datmo::initialize_tracking()
{
    // Initialize tracking before running track_a_person();
    frequency_   = frequency_init;
    uncertainty_ = uncertainty_init;
    associated_  = false;
    index_min_   = -1;
}

void Datmo::track_a_person()
{
    // see slides 29 to 42 related to tracking in the Datmo lecture
    float distance_min = std::numeric_limits<float>::infinity();
    associated_ = false;
    index_min_  = -1;

    // association between the tracked person and the possible detection
    for (int i = 0; i < nb_persons_detected_; ++i)
    {
        float current_dist = this->distancePoints(tracked_person_, detected_person_[i]);
        if (current_dist < distance_min)
        {
            distance_min = current_dist;
            index_min_   = i;
        }
    }

    // seuil d’association simple basé sur l’incertitude (ex: 1.0 m * uncertainty_)
    const float gate = 1.0f * std::max(uncertainty_min, std::min(uncertainty_max, uncertainty_));
    if (index_min_ >= 0 && distance_min <= gate)
    {
        associated_ = true;
        // update info knowing there is an association
        tracked_person_ = detected_person_[index_min_];
        frequency_ = std::min(frequency_max, frequency_ + 1);
        uncertainty_ = std::max(uncertainty_min, uncertainty_ - uncertainty_inc);
        // (publier éventuellement)
        // if (pub_tracking_) pub_tracking_->publish(tracked_person_);
    }
    else
    {
        // no association: prediction-only decay
        frequency_ = std::max(0, frequency_ - 1);
        uncertainty_ = std::min(uncertainty_max, uncertainty_ + uncertainty_inc);
        // si la fréquence tombe trop bas, on perd le tracking
        if (frequency_ == 0) associated_ = false;
    }
}

void Datmo::detect_and_track_a_person()
{
    // First run detection of moving persons in the current scan
    detect_a_moving_person();

    if (is_moving_person_detected_)
    {
        // A moving person was detected now
        if (!is_person_tracked_)
        {
            // Start tracking the newly detected moving person
            initialize_tracking();
            // initialize tracked_person_ with the detected moving person's position
            tracked_person_ = moving_detected_person_;
            // mark that we are now tracking a person
            is_person_tracked_ = true;
            // assume we have an association at start
            associated_ = true;
        }
        else
        {
            // We are already tracking a person -> try to associate/update using tracking logic
            track_a_person();
            // If tracking failed to associate, mark tracking as lost
            if (!associated_)
                is_person_tracked_ = false;
        }
    }
    else
    {
        // No moving person detected now
        if (is_person_tracked_)
        {
            // Try to keep tracking (prediction/update step) even without a current detection
            track_a_person();
            // If no association after tracking step, stop tracking
            if (!associated_)
                is_person_tracked_ = false;
        }
        // If not tracking and no detection -> nothing to do
    }
}

// ========== UTILITAIRES ==========

float Datmo::distancePoints(const geometry_msgs::msg::Point &pa, const geometry_msgs::msg::Point &pb)
{
    const double dx = static_cast<double>(pa.x) - static_cast<double>(pb.x);
    const double dy = static_cast<double>(pa.y) - static_cast<double>(pb.y);
    const double dz = static_cast<double>(pa.z) - static_cast<double>(pb.z);
    // lidar 2D: dz=0, mais on laisse 3D au cas où
    return static_cast<float>(std::sqrt(dx*dx + dy*dy + dz*dz));
}

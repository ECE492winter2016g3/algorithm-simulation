
#include "mapping.h"

#include <iostream>
#include <sstream>

#include <cstdio>
#include <cmath>
#include <cstring>

void mapping_init(map_state *state) {
	// Robot initially at midpoint of map
	state->x = state->y = 0.5f * BROAD_STEPS * BROAD_SIZE;
	state->theta = 0;

	// No segments initially
	state->all_segments_list = NULL;

	// Broad phase
	for (int x = 0; x < BROAD_STEPS; ++x) {
		for (int y = 0; y < BROAD_STEPS; ++y) {
			map_broad *tile = &state->broad_phase[x][y];

			// No points in list
			tile->point_list = NULL;

			// No segments in array
			tile->segment_array = (map_segment*)malloc(sizeof(map_segment*) * 8);
			tile->segment_array_size = 0;
			tile->segment_array_capacity = 8;
		}
	}

	// Pools initially empty
	state->segment_pool = NULL;
	state->point_pool = NULL;
}

/* Length of a vector */
float length(float x, float y) {
	return sqrt(x*x + y*y);
}

/* Normalize a vector of two floats */
void normalize(float *x, float *y) {
	float a = *x, b = *y;
	float len = length(a, b);
	*x = a / len;
	*y = b / len;
}

/* Dot product */
float dot(float a, float b, float x, float y) {
	return a*x + b*y;
}

float cross(float a, float b, float x, float y) {
	return a*y - b*x;
}

/* Angle between */
float angle_between(float a, float b, float x, float y) {
	float frac = dot(a, b, x, y) / (length(a, b) * length(x, y));
	// Correct floating point errors making the value slightly out of range
	if (frac > 1) frac = 1;
	if (frac < -1) frac = -1;
	return acos(frac);
}
float absangle_between(float a, float b, float x, float y) {
	float frac = dot(a, b, x, y) / (length(a, b) * length(x, y));
	// Correct floating point errors making the value slightly out of range
	if (frac > 1) frac = 1;
	if (frac < -1) frac = -1;
	float theta = acos(frac);
	return std::min(theta, 3.141592653f - theta);
}

/* Pool allocator for map_segments */
map_segment *alloc_segment(map_state *state) {
	if (!state->segment_pool) {
		state->segment_pool = (map_segment*)malloc(sizeof(map_segment));
		state->segment_pool->next = NULL;
	}
	map_segment *seg = state->segment_pool;
	state->segment_pool = seg->next;
	seg->next = NULL;
	seg->point_list = NULL;
	return seg;
}
void free_segment(map_state *state, map_segment *seg) {
	seg->next = state->segment_pool;
	state->segment_pool = seg;
}

/* Pool allocator for map_points */
map_point *alloc_point(map_state *state) {
	if (!state->point_pool) {
		state->point_pool = (map_point*)malloc(sizeof(map_point));
		state->point_pool->next = NULL;
	}
	map_point *pt = state->point_pool;
	state->point_pool = pt->next;
	pt->next = NULL;
	return pt;
}
void free_point(map_state *state, map_point *pt) {
	pt->next = state->point_pool;
	state->point_pool = pt;
}

/* Update a line segment's linear regression using it's points */
void regress_segment(map_state *state, map_segment *seg) {
	// TODO: Use protections onto the line of best fit in
	// order to get a more accurate length
	map_point *first = seg->point_list;
	map_point *last = first;
	while (last->next) last = last->next;
	// Linear least squares
	if (abs(last->y - first->y) < abs(last->x - first->x)) {
		// Linear least squares on Ys as a function of Xs
		int count = 0;
		float sumx = 0, sumy = 0, sumxy = 0, sumxx = 0;
		float greatestX = -100000;
		float smallestX = 100000;
		for (map_point *p = seg->point_list; p; p = p->next) {
			++count;
			sumx += p->x;
			sumy += p->y;
			sumxy += p->x * p->y;
			sumxx += p->x * p->x;
			greatestX = std::max(greatestX, p->x);
			smallestX = std::min(smallestX, p->x);
		}
		float slope = (sumx*sumy - count*sumxy) / (sumx*sumx - count*sumxx);
		float interc = (sumy - slope*sumx) / count;
		// Now Y = interc + X*slope
		seg->ox = smallestX;
		seg->oy = interc + smallestX*slope;
		seg->vx = (greatestX - smallestX);
		seg->vy = (greatestX - smallestX)*slope;
	} else {
		// Linear least squares on Xs as a function of Ys
		int count = 0;
		float sumx = 0, sumy = 0, sumxy = 0, sumxx = 0;
		float greatestY = -100000;
		float smallestY = 100000;
		for (map_point *p = seg->point_list; p; p = p->next) {
			++count;
			sumx += p->y;
			sumy += p->x;
			sumxy += p->y * p->x;
			sumxx += p->y * p->y;
			greatestY = std::max(greatestY, p->y);
			smallestY = std::min(smallestY, p->y);
		}
		float slope = (sumx*sumy - count*sumxy) / (sumx*sumx - count*sumxx);
		float interc = (sumy - slope*sumx) / count;
		// Now  X = interc + Y*slope
		seg->oy = smallestY;
		seg->ox = interc + smallestY*slope;
		seg->vy = (greatestY - smallestY);
		seg->vx = (greatestY - smallestY)*slope;
	}
}

/* Do feature extraction */
void feature_extract(map_state *state, 
	map_point **points, map_segment **segments, 
	int count, float angles[], float distances[]) 
{
	// First, extract the global X/Y coordinates
	float xs[50];
	float ys[50];
	for (int i = 0; i < count; ++i) {
		xs[i] = state->x + cos(angles[i] + state->theta)*distances[i];
		ys[i] = state->y + sin(angles[i] + state->theta)*distances[i];
	}
	std::stringstream ss;
	ss << "convert -size 1000x1000 xc:white -fill black -stroke black ";
	for (int i = 0; i < count; ++i) {
		ss << "-draw \"circle " 
			<< (xs[i]-1) << "," << (ys[i]-1) << " " 
			<< (xs[i]+1) << "," << (ys[i]+1) << "\" ";
	}


	// List of point and segments outputed
	map_segment *seg_result = NULL;
	map_point *pt_result = NULL;

	// Now, starting from the first point, try to generate line segments
	int start_index = 0; // First index in the line segment
	int start_is_in_segment = 0; // First index was already used in a segment
	float lastDifference = length(xs[0] - xs[1], ys[0] - ys[1]);
	for (int i = 1; i <= count; ++i) {
		//std::cout << "Point " << i << " from start " << start_index << "\n";
		// Use this point as the end point, starting at the first index, and
		// see if all the other points are near the line segment (use
		// perpendicular distance to the segment)
		float thisDifference;
		if (i < count) {
			thisDifference = length(xs[i] - xs[i-1], ys[i] - ys[i-1]);
		}
		int failed;
		if (i == count) {
			// Final iteration, always fail
			failed = 1;
		} else if (thisDifference > 1.6*lastDifference
			|| thisDifference < 0.625*lastDifference) {
			// Not following a smooth wall
			failed = 1;
		} else {
			// Normal iteration behavior
			float ux = xs[i] - xs[start_index];
			float uy = ys[i] - ys[start_index];
			normalize(&ux, &uy);
			float tx = xs[i] - state->x; // Direction from robot to edge
			float ty = ys[i] - state->y;
			if (absangle_between(ux, uy, tx, ty) < 0.5) {
				// Line seg is at too great an angle to us, don't use it
				failed = 1;
			} else {
				failed = 0;
				// See if the line segment is properly ordered
				for (int j = start_index + 1; j < i; ++j) {
					float dx = xs[j] - xs[start_index];
					float dy = ys[j] - ys[start_index];
					float projLen = dot(ux, uy, dx, dy);
					dx -= projLen*ux;
					dy -= projLen*uy;
					// 1.5 -> allow plus variance on both endpoints
					// and minus variance on inner points to still register
					// as a segment.
					//std::cout << "length: " << length(dx, dy) << "\n";
					if (length(dx, dy) > 3.0f * LIDAR_VARIANCE) {
						failed = 1;
						break;
					}
				}
			}
		}
		lastDifference = thisDifference;

		// On a fail, if we have a line segment of >3 points already, emit it
		// Otherwise, emit the start_index point and advance start_index.
		if (failed) {
			if ((i - start_index) > 2) {
				// Emit the points start_index up to i - 1
				// (i is the index that we failed to extend to)
				map_segment *seg = alloc_segment(state);
				for (int j = start_index; j < i; ++j) {
					map_point *pt = alloc_point(state);
					pt->x = xs[j];
					pt->y = ys[j];
					pt->next = seg->point_list;
					seg->point_list = pt;
				}
				regress_segment(state, seg);
				seg->next = seg_result;
				seg_result = seg;
				std::cout << "Allocated segment (" << (i - start_index) << ")\n";
				std::cout << "  at (" 
					<< xs[start_index] << ", " << ys[start_index] << ")\n"
					<< "  dir ("
					<< (xs[i-1] - xs[start_index]) << ", "
					<< (ys[i-1] - ys[start_index]) << ")\n";

				// New start index is i *minus one*, because the end point
				// of one segment can also be the start point of a new one
				start_index = i - 1;
				start_is_in_segment = 1;
			} else if (start_is_in_segment) {
				// Just advance, that point does not need to be emitted as it
				// is already in a line segment
				++start_index;
				start_is_in_segment = 0;
			} else {
				// Emit the start_index point individually and advance the 
				// start index by one.
				map_point *pt = alloc_point(state);
				pt->x = xs[start_index];
				pt->y = ys[start_index];
				pt->next = pt_result;
				pt_result = pt;
				std::cout << "Allocated point\n";

				// New start index one ahead
				++start_index;
				// start_is_in_segment = 0; -> already implied branch's conditions
			}
		} else {
			// Nothing to do, just keep extending the current line segment
		}
	}

	for (map_segment *seg = seg_result; seg; seg = seg->next) {
		ss << "-draw \"line " 
			<< (seg->ox) << "," << (seg->oy) << " " 
			<< (seg->ox + seg->vx) << "," << (seg->oy + seg->vy) << "\" ";	
		ss << "-fill red -draw \"circle " 
			<< (seg->ox-1) << "," << (seg->oy-1) << " " 
			<< (seg->ox+1) << "," << (seg->oy+1) << "\" ";	
	}
	ss << "int.png";
	system(ss.str().c_str());

	// Output the results
	*points = pt_result;
	*segments = seg_result;
}


// Delete features
void delete_features(map_state *state, map_point *points, map_segment *segments) {
	for (map_point *point = points; point; point = point->next) {
		// Free here won't clobber next, so it's fine in the iteration
		free_point(state, point);
	}
	for (map_segment *segment = segments; segment; segment = segment->next) {
		for (map_point *point = segment->point_list; point; point = point->next) {
			free_point(state, point);
		}
		free_segment(state, segment);
	}
}


// Update a set of features based on a change in position / rotation of the robot
// (Subtract, rotate, add)
void update_features(map_state *state, float oldX, float oldY, float oldTheta,
	map_point *points, map_segment *segments) {
	std::cout << "update_features(" << (state->x - oldX) << ", " << (state->y - oldY) << ")\n";

	// Change
	float dTheta = state->theta - oldTheta;
	float st = sin(dTheta);
	float ct = cos(dTheta);

	// Subtract
	for (map_segment *segment = segments; segment; segment = segment->next) {
		for (map_point *point = segment->point_list; point; point = point->next) {
			point->x -= oldX;
			point->y -= oldY;
		}
		segment->ox -= oldX;
		segment->oy -= oldY;
	}
	for (map_point *point = points; point; point = point->next) {
		point->x -= oldX;
		point->y -= oldY;
	}

	// Rotate
	for (map_segment *segment = segments; segment; segment = segment->next) {
		for (map_point *point = segment->point_list; point; point = point->next) {
			float x = point->x;
			float y = point->y;
			point->x = x*ct - y*st;
			point->y = x*st + y*ct;
		}
		float ox = segment->ox;
		float oy = segment->oy;
		segment->ox = ox*ct - oy*st;
		segment->oy = ox*st + oy*ct;
		float vx = segment->vx;
		float vy = segment->vy;
		segment->vx = vx*ct - vy*st;
		segment->vy = vx*st + vy*ct;
	}
	for (map_point *point = points; point; point = point->next) {
		float x = point->x;
		float y = point->y;
		point->x = x*ct - y*st;
		point->y = x*st + y*ct;
	}

	// Add
	for (map_segment *segment = segments; segment; segment = segment->next) {
		for (map_point *point = segment->point_list; point; point = point->next) {
			point->x += state->x;
			point->y += state->y;
		}
		segment->ox += state->x;
		segment->oy += state->y;
	}
	for (map_point *point = points; point; point = point->next) {
		point->x += state->x;
		point->y += state->y;
	}
}


// Tweak the rotation of the robot by a small amount to best match 
void rotation_tweak(map_state *state, map_segment *segments) {
	// For each segment in the list, try to find a segment in the map that
	// has a very similar rotation, and is somewhat nearby.

	float totx = 0, toty = 0; // Total correction to make to the robot unit vector
	int totcount = 0;

	for (map_segment *to_match = segments; to_match; to_match = to_match->next) {
		// Find a similarly rotated segment
		map_segment *best_match = NULL;
		float best_theta = 10000.0f;
		// TODO: Only search segments that are realistic candidates
		for (map_segment *a_match = state->all_segments_list; a_match; a_match = a_match->next) {
			float angle = angle_between(a_match->vx, a_match->vy, to_match->vx, to_match->vy);
			if (angle < best_theta) {
				best_theta = angle;
				best_match = a_match;
			}
		}

		// If there is a match within the turn tweak amount, get the delta
		if (best_theta < TURN_TWEAK) {
			// Let <a,b> = unit vector to turn towards
			float a = best_match->vx, b = best_match->vy;
			if (dot(a, b, to_match->vx, to_match->vy) < 0) {
				a = -a;
				b = -b;
			}
			normalize(&a, &b);

			// Let <x,y> = to match's unit direction
			float x = to_match->vx, y = to_match->vy;
			normalize(&x, &y);

			// Add the difference to the modifier
			totx += (a - x);
			toty += (b - y);
			++totcount;
		}
	}

	// Modify the robot rotation
	float oldRot = state->theta;
	float ux = cos(state->theta);
	float uy = sin(state->theta);
	normalize(&ux, &uy);
	ux += totx / totcount;
	uy += toty / totcount;
	state->theta = atan2(uy, ux);
	printf("Rotation tweaked: %.2f -> %.2f\n", oldRot, state->theta);
}


// Process linear motion (post rotation tweaking)
// Returns: 0 - success
//          -1 - couldn't reckon position, don't add geometry
int linearmotion_process(map_state *state, map_segment *segments) {
	// Histogram of differences
	typedef struct {
		float weight;
		float diff;
	} diff_entry;
	diff_entry diff_array[10] = {0};
	int diff_cap = 10;
	int diff_count = 0;

	// Dir of movement
	float ux = cos(state->theta);
	float uy = sin(state->theta);

	for (map_segment *to_match = segments; to_match; to_match = to_match->next) {
		std::cout << "Lin motion to match...\n";
		// Try to match to a segment with a very similar rotation, and 
		// a small linear difference along the direction of supposed motion
		// (the current theta)
		for (map_segment *a_match = state->all_segments_list; a_match; a_match = a_match->next) {
			if (absangle_between(to_match->vx, to_match->vy, a_match->vx, a_match->vy) < TURN_TWEAK) {
				// std::cout << "Match " << to_match->vx << ", " << to_match->vy
				// 	<< "(" << to_match->ox << "," << to_match->oy << ")"
				// 	<< " to " << a_match->vx << ", " << a_match->vy
				// 	<< "(" << a_match->ox << "," << a_match->oy << ")" <<"\n";
				// Is a candidate, compute the distance to it
				// First compute the perpendicular vector from segment to match 
				// = (match.o - seg.o) projected onto (perp seg.v, in the direction of movement) 
				float dx = a_match->ox - to_match->ox;
				float dy = a_match->oy - to_match->oy;
				float perpx = -to_match->vy;
				float perpy = +to_match->vx;
				// Make sure perp is in the direction of movement
				if (dot(perpx, perpy, ux, uy) < 0) {
					perpx = -perpx;
					perpy = -perpy;
				}
				normalize(&perpx, &perpy);
				// Do the projection
				float perplen = dot(dx, dy, perpx, perpy);
				// Now project the direction of movement onto the perpendicular vector
				float frac = dot(perpx, perpy, ux, uy);
				// Now scale that up for the final movement along the direction of travel
				float dirlen = perplen / frac;

				// Note: When frac is small, dirlen will be large, and errors will be
				// scaled up, so use frac as the weight of a measurement
				// Add the dirlen to the histogram
				if (frac > 0.2) {
					// 0.2 -> threshold of useful data
					// below that the wall is almost paralell to our movement
					// and we can't get a distance delta from it
					std::cout << "Lin motion candidate weight " << frac << 
						" distance " << dirlen << "\n";
					int added = 0;
					int leastWeightIndex;
					float leastWeight = 10000;
					for (int i = 0; i < diff_count; ++i) {
						if (abs(diff_array[i].diff - dirlen) < 3*LIDAR_VARIANCE) {
							diff_array[i].weight += frac;
							added = 1;
							break;
						}
						if (diff_array[i].weight < leastWeight) {
							leastWeight = diff_array[i].weight;
							leastWeightIndex = i;
						}
					}
					if (!added) {
						// Add a new entry
						if (diff_count < diff_cap) {
							// Add a new entry
							diff_array[diff_count].weight = frac;
							diff_array[diff_count].diff = dirlen;
							++diff_count;
						} else {
							// Try replacing the least weight entry
							if (frac > leastWeight) {
								diff_array[leastWeightIndex].weight = frac;
								diff_array[leastWeightIndex].diff = dirlen;
							}
						}
					}
				}
			}
		}
	}

	// Now find the highest weight item in the histogram and move the robot's
	// reckoned position by that much
	if (diff_cap == 0) {
		printf("Error: No features found to reckon based on");
		return -1;
	} else {
		int best_index;
		float best_weight = 0;
		for (int i = 0; i < diff_cap; ++i) {
			if (diff_array[i].weight >= best_weight) {
				best_weight = diff_array[i].weight;
				best_index = i;
			}
		}
		float best_diff = diff_array[best_index].diff;
		state->x += ux * best_diff;
		state->y += uy * best_diff;
		printf("linearmotion moved by %.2f\n", best_diff);
		return 0;
	}
}


// Get the broad phase tile coordinates for a given point
void broad_get_coord(float fx, float fy, int *ix, int *iy) {
	int x = (int)(fx / BROAD_SIZE);
	int y = (int)(fy / BROAD_SIZE);
	if (x < 0) x = 0;
	if (x >= BROAD_STEPS) x = BROAD_STEPS - 1;
	if (y < 0) y = 0;
	if (y >= BROAD_STEPS) y = BROAD_STEPS - 1;
	*ix = x;
	*iy = y;
}


// Broad phase insert point
void broad_insert_point(map_state *state, map_point *pt) {
	int x, y;
	broad_get_coord(pt->x, pt->y, &x, &y);
	map_broad *tile = &state->broad_phase[x][y];
	pt->next = tile->point_list;
	tile->point_list = pt;
}


// Merge in a line segment adding it to the state
void merge_segment(map_state *state, map_segment *seg) {
	// Try to find a colinear line segment to merge with
	int merged = 0;
	for (map_segment *cand = state->all_segments_list; cand; cand = cand->next) {
		// std::cout << "Merging: \n";
		// std::cout << "  Seg: " << seg->ox << ", " << seg->oy << ", " << seg->vx << ", " << seg->vy << "\n"
		// 	<< "  Into: " << cand->ox << ", " << cand->oy << ", " << cand->vx << ", " << cand->vy << "\n";
		// First, point the two segments in the same direction
		if (dot(cand->vx, cand->vy, seg->vx, seg->vy) < 0) {
			seg->ox += seg->vx;
			seg->oy += seg->vy;
			seg->vx = -seg->vx;
			seg->vy = -seg->vy;
		}

		// Discard candidate if they are too dissmilar in direction
		float angleDiff = absangle_between(cand->vx, cand->vy, seg->vx, seg->vy);
		if (angleDiff > 0.5) {
			// Don't bother doing work in that case
			continue;
		}

		// Project each end of seg onto cand and see if there is
		// an intersection
		float ux = cand->vx, uy = cand->vy;
		normalize(&ux, &uy);
		float p1 = dot(seg->ox - cand->ox, seg->oy - cand->oy, 
			ux, uy);
		float d1 = length(
			seg->ox - (cand->ox + ux*p1),
			seg->oy - (cand->oy + uy*p1)); 
		float p2 = dot(seg->ox + seg->vx - cand->ox, seg->oy + seg->vy - cand->oy,
			ux, uy);
		float d2 = length(
			seg->ox + seg->vx - (cand->ox + ux*p2),
			seg->oy + seg->vy - (cand->oy + uy*p2));

		// Intersection
		float denom = cross(seg->vx, seg->vy, cand->vx, cand->vy);
		float t = cross(cand->ox - seg->ox, cand->oy - seg->oy, cand->vx, cand->vy) / denom;
		float u = cross(cand->ox - seg->ox, cand->oy - seg->oy, seg->vx, seg->vy) / denom;
		int doesIntersect = ((denom != 0) && (u > 0 && u < 1) && (t > 0 && t < 1));

		// Relatively close test
		float avglen = 0.5*(length(seg->vx, seg->vy) + length(cand->vx, cand->vy));
		int relativelyClose = 
			(d1 < 0.1*avglen) && (d2 < 0.2*avglen) ||
			(d1 < 0.2*avglen) && (d2 < 0.1*avglen);

		float tlen = length(cand->vx, cand->vy);
		// Is one of the ends of the line seg touching the candidate?
		int isTouching =  (d2 < 2*LIDAR_VARIANCE || d1 < 2*LIDAR_VARIANCE);
		// Do the line segments overlap in projection?
		int projectionDoesOverlap = (p2 > 0 && p2 < tlen) || (p1 > 0 && p1 < tlen);
		// 
		if ((projectionDoesOverlap && (isTouching || relativelyClose)) || doesIntersect) {
			// We have an intersection. Add the points from seg to
			// cand and re-regress it.
			// Save the last and second last point of cand
			map_point *second_last = cand->point_list;
			map_point *last = second_last->next;
			while (last->next) {
				second_last = last;
				last = last->next;
			}

			// Now insert the new points into the second last plac
			// in the candidate
			second_last->next = seg->point_list;
			map_point* last_to_insert = seg->point_list;
			while (last_to_insert->next) last_to_insert = last_to_insert->next;
			last_to_insert->next = last;

			// Re-regress
			regress_segment(state, cand);

			// Done
			merged = 1;
			printf(" ===> Merged\n");
			break;
		}
	}

	// Couldn't merge? Add as a new global segment
	if (!merged) {
		printf(" ==> New segment\n");
		seg->next = state->all_segments_list;
		state->all_segments_list = seg;
	}
}


// Merge in a point adding it to the map state
void merge_point(map_state *state, map_point *point) {
	// TODO: Try to merge with edges
	broad_insert_point(state, point);
}

// Merge new geometry into the map state
void merge_geometry(map_state *state, map_point *points, map_segment *segments) {
	// Attatch the segments
	while (segments) {
		map_segment *seg = segments->next;
		merge_segment(state, segments);
		segments = seg;
	}

	// Attach the points
	while (points) {
		map_point *next = points->next;
		merge_point(state, points);
		points = next;
	}
}


void mapping_update_lin(map_state *state, int count, float angles[], float distances[]) {
	// First step is feature extraction, we need to break down the
	// sensor input into points and segments (sequences of 3+ colinear points)
	map_point *points;
	map_segment *segments;
	feature_extract(state, &points, &segments, count, angles, distances);

	// Now, we approach linear movement in three steps:
	// 1) Do fine adjustment of the rotation of the robot, assuming that
	//    any minor rotations made during linear movement were quite small.
	// 2) Once our rotation has been corrected, match up line segments to
	//    to get a histogram of expected movements, and pick a heavily
	//    median weighted actual movement from those 
	// 3) With our updated position, merge the new geometry into the
	//    the global map state
	float oldX = state->x;
	float oldY = state->y;
	float oldTheta = state->theta;
	rotation_tweak(state, segments);
	update_features(state, oldX, oldY, oldTheta, points, segments);
	//
	oldX = state->x; oldY = state->y; oldTheta = state->theta;
	if (linearmotion_process(state, segments) == 0) {
		// If we could process the linear motion, update the map
		update_features(state, oldX, oldY, oldTheta, points, segments);
		//
		merge_geometry(state, points, segments);
	} else {
		delete_features(state, points, segments);
	}
}


// Process linear motion (post rotation tweaking)
int angularmotion_process(map_state *state, map_segment *segments, float turnHint) {
	// Histogram of differences
	typedef struct {
		float weight;
		float diff;
		float total;
		int count;
	} diff_entry;
	diff_entry diff_array[10] = {0};
	std::memset(diff_array, 0, sizeof(diff_entry)*10);
	int diff_cap = 10;
	int diff_count = 0;

	for (map_segment *to_match = segments; to_match; to_match = to_match->next) {
		//std::cout << "Rot motion to match...\n";
		// Find the perpendicular distance to the robot
		// by projecting the robot position on to the edge
		float pdist1;
		float psign1;
		{
			float dx = state->x - to_match->ox;
			float dy = state->y - to_match->oy;
			float elen = length(to_match->vx, to_match->vy);
			float frac = dot(dx, dy, to_match->vx, to_match->vy) / (elen * elen);
			psign1 = frac;
			pdist1 = length(dx - frac*to_match->vx, dy - frac*to_match->vy);
		}
		float theta1 = atan2(to_match->vy, to_match->vx);
	
		// Try to match to a segment with a very similar perpendicular distance
		// to the robot
		for (map_segment *a_match = state->all_segments_list; a_match; a_match = a_match->next) {
			float pdist2;
			float psign2;
			{
				float dx = state->x - a_match->ox;
				float dy = state->y - a_match->oy;
				float elen = length(a_match->vx, a_match->vy);
				float frac = dot(dx, dy, a_match->vx, a_match->vy) / (elen * elen);
				psign2 = frac;
				pdist2 = length(dx - frac*a_match->vx, dy - frac*a_match->vy);
			}
			if (abs(pdist1 - pdist2) < 3*LIDAR_VARIANCE) {
				// std::cout << "Match " << to_match->vx << ", " << to_match->vy
				// 	<< "(" << to_match->ox << "," << to_match->oy << ")"
				// 	<< " to " << a_match->vx << ", " << a_match->vy
				// 	<< "(" << a_match->ox << "," << a_match->oy << ")" <<"\n";

				// Is a candidate, compute the angluar difference between the two
				float theta2 = atan2(a_match->vy, a_match->vx);
				float dtheta = theta2 - theta1;
				// Force positive difference
				if (dtheta < 0)
					dtheta += 2*3.141592653f;

				// Now, we have to calculate how reliable the measurement is, off
				// of how oblique the distance to the feature is
				// TODO: Implement, for now just weights everything 1
				float weight = 1.0f;

				// Note: When frac is small, dirlen will be large, and errors will be
				// scaled up, so use frac as the weight of a measurement
				// Add the dirlen to the histogram
				if (weight > 0.2) {
					// 0.2 -> threshold of useful data
					// below that the wall is almost paralell to our movement
					// and we can't get a distance delta from it
					std::cout << "Rot motion candidate weight " << weight << 
						" dtheta " << dtheta << "\n";
					int added = 0;
					int leastWeightIndex;
					float leastWeight = 10000;
					for (int i = 0; i < diff_count; ++i) {
						float del = diff_array[i].diff - dtheta;
						if (del < 0)
							del += 2*3.141592653f;

						if (std::min(del, del) < 2*TURN_TWEAK) {
							diff_array[i].weight += weight;
							diff_array[i].total += dtheta;
							diff_array[i].count += 1;
							added = 1;
							break;
						}
						if (diff_array[i].weight < leastWeight) {
							leastWeight = diff_array[i].weight;
							leastWeightIndex = i;
						}
					}
					if (!added) {
						// Add a new entry
						if (diff_count < diff_cap) {
							// Add a new entry
							diff_array[diff_count].weight = weight;
							diff_array[diff_count].diff = dtheta;
							diff_array[diff_count].total = dtheta;
							diff_array[diff_count].count = 1;
							++diff_count;
						} else {
							// Try replacing the least weight entry
							if (weight > leastWeight) {
								diff_array[leastWeightIndex].weight = weight;
								diff_array[leastWeightIndex].diff = dtheta;
								diff_array[leastWeightIndex].total = dtheta;
								diff_array[leastWeightIndex].count = 1;
							}
						}
					}
				}
			}
		}
	}
	//

	// Now find the highest weight item in the histogram and move the robot's
	// reckoned position by that much
	if (diff_cap == 0) {
		printf("Error: No features found to reckon based on");
		return -1;
	} else {
		int found_within_hint = 0;
		int best_index;
		float best_weight = 0;
		for (int i = 0; i < diff_cap; ++i) {
			if (diff_array[i].weight >= best_weight && (diff_array[i].diff - turnHint) < 1) {
				best_weight = diff_array[i].weight;
				best_index = i;
				found_within_hint = 1;
			}
		}
		if (found_within_hint) {
			float best_diff = diff_array[best_index].total / diff_array[best_index].count;
			state->theta += best_diff;
			printf("%f, %f, %d\n", diff_array[best_index].diff, diff_array[best_index].total, diff_array[best_index].count);
			printf("rotationalmotion moved by %.2f\n", best_diff);
			return 0;
		} else {
			printf("Error: rotationalmotion not found within hint\n");
		}
	}
}


void mapping_update_rot(map_state *state, int count, float angles[], float distances[], float turnHint) {
	// First step is feature extraction, we need to break down the
	// sensor input into points and segments (sequences of 3+ colinear points)
	map_point *points;
	map_segment *segments;
	feature_extract(state, &points, &segments, count, angles, distances);

	// Now, we approach rotational movement in two steps
	// 0) The position is assumed to be unchanged (That is, we can accurately
	//    rotate the robot without issue.
	// 1) We should see fairly similar geometry to what we did before the
	//    rotation, just at different points angularly (modulo the sampling
	//    frequency). That is, we should see line segments with similar
	//    perpendicular distances to the robot, but different angles in the
	//    global space. Try to find the median angle to rotate by from those
	//    anglular differences.
	// 2) With our updated rotation, merge the new geometry into the global
	//    map state.
	float oldX = state->x;
	float oldY = state->y;
	float oldTheta = state->theta;
	if (angularmotion_process(state, segments, turnHint) == 0) {
		// Success, update map
		update_features(state, oldX, oldY, oldTheta, points, segments);
		//
		merge_geometry(state, points, segments);
	} else {
		// Failure, don't know where we are
		delete_features(state, points, segments);
	}
}


void mapping_initial_scan(map_state *state, int count, float angles[], float distances[]) {
	map_point *points;
	map_segment *segments;
	feature_extract(state, &points, &segments, count, angles, distances);
	merge_geometry(state, points, segments);
}


void mapping_dump(map_state *state, const char *filename) {
	std::stringstream ss;
	map_segment *seg = state->all_segments_list;
	ss << "convert -size 1000x1000 xc:white -fill black -stroke black ";
	while (seg) {
		ss << "-draw \"line " 
			<< seg->ox << "," << seg->oy << " " 
			<< (seg->ox + seg->vx) << "," << (seg->oy + seg->vy) << "\" ";\
		seg = seg->next;
	}
	ss << "-draw \"circle " 
		<< (state->x-1) << "," << (state->y-1) << " "
		<< (state->x+1) << "," << (state->y+1) << "\" ";
	ss << "-stroke blue -draw \"line " 
		<< (state->x) << "," << (state->y) << " "
		<< (state->x+cos(state->theta)*10) << "," << (state->y+sin(state->theta)*10) << "\" ";
	ss << filename;
	system(ss.str().c_str());
	// std::stringstream ss2;
	// ss2 << "mv int.png int_" << filename;
	// system(ss2.str().c_str());
}
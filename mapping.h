
#include <stdlib.h>
#include <cstddef>

/* Broad phase is a 10x10 grid of 100x100 cm squares */
#define BROAD_STEPS 10
#define BROAD_SIZE 100.0f

/* Variance in the lidar range from the true value is +- 1cm */
#define LIDAR_VARIANCE 1.5f

/* Max number of samples in a sweep by the sensor */
#define SWEEP_COUNT 50

/* Maximum amount the robot can turn by during linear movement (1 / 30th turn | 12 deg) */
#define TURN_TWEAK (3.14159f * 2.0f / 30.0f)

typedef struct map_point {
	float x, y;
	struct map_point *next;
} map_point;

typedef struct map_segment {
	/* Intrusive list of all points that are part of the segment */
	map_point *point_list;

	/* Next in intrusive list of segments */
	struct map_segment *next;

	/* Cached data about the segment, calculated 
	 * using an approximate linear regression of the points */
	float ox, oy; // Origin
	float vx, vy; // Direction + Length
} map_segment;

typedef struct {
	/* Intrusive list of points in the broad phase cell*/
	map_point *point_list;

	/* Array of all segments crossing through the cell */
	map_segment *segment_array;
	size_t segment_array_size;
	size_t segment_array_capacity;
} map_broad;

typedef struct {
	/* Robot position */
	float x;
	float y;
	float theta;

	/* Intrusive list of all line segments */
	map_segment *all_segments_list;

	/* Board phase entries */
	map_broad broad_phase[BROAD_STEPS][BROAD_STEPS];

	/* Pool of free points and segments */
	map_segment *segment_pool;
	map_point *point_pool;
} map_state;

/* Initialize a map state struct */
void mapping_init(map_state *state);

/* Move the robot by a */
void mapping_moveahead(map_state *state, float dist_estimate);

/* Rotate the robot by an angle estimate */
void mapping_rotate(map_state *state, float angle_estimate);

/* Update the map with a new scan after linear movement */
void mapping_update_lin(map_state *state, int count, float angles[], float distances[]);

/* Update the map with a new scan after rotational movement */
void mapping_update_rot(map_state *state, int count, float angles[], float distances[], float turnHint);

/* Initial scan at origin */
void mapping_initial_scan(map_state *state, int count, float angles[], float distances[]);


void mapping_dump(map_state *state, const char *filename);
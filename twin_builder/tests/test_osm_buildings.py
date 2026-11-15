"""OSM building footprints, with Overpass mocked."""
import json
import math
import unittest
import urllib.error
from unittest import mock

from twin_builder import osm_buildings as osm


def _ll(lat, lon, east, north):
    dlat = north / 111320.0
    dlon = east / (111320.0 * math.cos(math.radians(lat)))
    return {"lat": lat + dlat, "lon": lon + dlon}


class OsmBuildingTests(unittest.TestCase):
    def test_rectangle_becomes_a_sized_building(self):
        lat, lon = 33.0, -112.0
        geom = [_ll(lat, lon, e, n) for e, n in ((-10, -5), (10, -5), (10, 5), (-10, 5), (-10, -5))]
        element = {
            "type": "way", "id": 9,
            "tags": {"building": "yes", "building:levels": "2"},
            "geometry": geom,
        }
        box = {"west": lon - 0.01, "south": lat - 0.01, "east": lon + 0.01, "north": lat + 0.01}
        prop = osm._way_to_prop(element, box)
        self.assertEqual(prop["class"], "building")
        self.assertAlmostEqual(prop["lat"], lat, places=4)
        self.assertAlmostEqual(prop["size_m"][0], 20.0, delta=1.0)
        self.assertAlmostEqual(prop["size_m"][1], 10.0, delta=1.0)
        self.assertAlmostEqual(prop["size_m"][2], 6.0, delta=0.01)
        self.assertEqual(prop["source"], "osm")

    def test_box_is_centered_on_the_footprint_not_the_vertex_average(self):
        lat, lon = 33.0, -112.0
        # A right triangle. The vertex average is not the rectangle centre.
        geom = [_ll(lat, lon, e, n) for e, n in ((0, 0), (20, 0), (0, 10), (0, 0))]
        element = {"type": "way", "id": 4, "tags": {"building": "yes", "height": "30 ft"}, "geometry": geom}
        box = {"west": lon - 0.01, "south": lat - 0.01, "east": lon + 0.01, "north": lat + 0.01}
        prop = osm._way_to_prop(element, box)
        center = _ll(lat, lon, 10, 5)
        self.assertAlmostEqual(prop["lat"], center["lat"], places=5)
        self.assertAlmostEqual(prop["lon"], center["lon"], places=5)
        self.assertAlmostEqual(prop["size_m"][2], 9.144, delta=0.01)

    def test_osm_buildings_replace_vision_buildings(self):
        vision = [
            {"class": "building", "lat": 1, "lon": 2},
            {"class": "tree", "lat": 1, "lon": 2},
        ]
        osm_rows = [{"class": "building", "lat": 3, "lon": 4, "source": "osm"}]
        merged = osm.merge_buildings(vision, osm_rows)
        self.assertEqual([p["class"] for p in merged], ["building", "tree"])
        self.assertEqual(merged[0]["source"], "osm")
        self.assertEqual(osm.merge_buildings(vision, []), vision)

    def test_query_uses_the_patch_and_parses_ways(self):
        body = json.dumps({"elements": [{
            "type": "way", "id": 3,
            "tags": {"building": "house", "height": "9"},
            "geometry": [_ll(33.0, -112.0, e, n) for e, n in ((-8, -6), (8, -6), (8, 6), (-8, 6), (-8, -6))],
        }]}).encode()

        class Resp:
            def read(self):
                return body
            def __enter__(self):
                return self
            def __exit__(self, *args):
                return False

        with mock.patch.object(osm.urllib.request, "urlopen", return_value=Resp()):
            rows = osm.buildings_in_bbox({
                "west": -112.01, "south": 32.99, "east": -111.99, "north": 33.01,
            })
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]["size_m"][2], 9.0, delta=0.01)

    def test_network_failure_is_an_empty_list(self):
        with mock.patch.object(osm.urllib.request, "urlopen", side_effect=urllib.error.URLError("down")):
            self.assertEqual(osm.buildings_in_bbox({
                "west": -112.01, "south": 32.99, "east": -111.99, "north": 33.01,
            }), [])


if __name__ == "__main__":
    unittest.main()

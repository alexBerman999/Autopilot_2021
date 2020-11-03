import pytest

from unitconv import Lat_Long_Converter

def test_shouldConvertLatitudeLongitudeToMeters():
	converter = Lat_Long_Converter(orig=(35, 70))
	point = converter.latLongPointToMetric((35.025, 70.05))
	assert abs(point[0] - 4551) < 5 and abs(point[1] - 2773) < 5
	
def test_metricPointToLatLong():
	converter = Lat_Long_Converter(orig=(35, 70))
	point = converter.metricPointToLatLong((4551, 2773))
	assert abs(point[0] - 35.025) < 5 and abs(point[1] - 70.05) < 5

def test_setOrigin():
	converter = Lat_Long_Converter(orig=(35, 70))
	point1 = converter.metricPointToLatLong((4551, 2773))
	converter.setOrigin((36, 71))
	point2 = converter.metricPointToLatLong((4551, 2773))
	assert point1 != point2

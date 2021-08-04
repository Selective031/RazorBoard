/*
 * signature.h
 *
 *  Created on: Mar 25, 2021
 *      Author: Carl Wallmark
 */

#ifndef INC_BWF_H_
#define INC_BWF_H_

#define ADC_SAMPLE_LEN			512			// How many samples taken for ADC Stream
#define SIGNATURE_SAMPLE_LEN	256			// How many samples taken for Signature
#define SIGNATURE_LEN			128			// Length of the signature

int16_t ADCADC[ADC_SAMPLE_LEN];
int16_t ADCADC_REAR[ADC_SAMPLE_LEN];
int16_t ADC_BUFFER[ADC_SAMPLE_LEN];
int16_t ADC_REAR_BUFFER[ADC_SAMPLE_LEN];
float32_t validSignature[SIGNATURE_LEN] =


{ 		214.871185,482.408661,653.609131,736.649292,744.055664,699.717102,634.489746,559.587341,
		481.410034,402.946167,274.879150,1.859665,-456.666931,-984.747070,-1400.364014,-1609.898560,
		-1647.239990,-1580.859863,-1457.802612,-1306.777954,-1145.464966,-983.490051,-825.570007,-674.374756,
		-532.315796,-400.884979,-280.461151,-170.435318,-70.510658,19.447758,122.199738,323.726929,700.389709,
		1208.363037,1672.670898,1952.999146,2033.851562,1945.888550,1653.926025,1135.521729,493.994507,-78.084106,
		-456.845459,-648.360107,-674.045715,-489.999023,-78.370308,441.802979,879.352356,1123.917725,1195.381592,
		1160.521240,1067.949341,946.498779,813.452881,678.432617,547.189697,414.493988,207.338837,-165.935196,
		-702.734619,-1241.603760,-1616.620850,-1779.445068,-1772.155640,-1573.519043,-1142.961914,-539.824768,
		46.724350,460.356567,673.637939,751.147949,749.207092,704.644043,639.792725,565.540833,487.172699,408.613647,
		284.503418,20.120682,-429.823822,-956.970825,-1379.570312,-1598.650635,-1642.885010,-1581.367432,-1461.933105,
		-1313.244751,-1152.497925,-990.203491,-831.739441,-679.963745,-536.774536,-404.230591,-283.248474,-173.449951,
		-74.215157,13.023215,109.252579,302.061005,673.575073,1183.548096,1654.672974,1942.188721,2028.730957,
		1949.208862,1670.036133,1163.492310,523.517944,-56.682030,-445.631226,-643.312378,-675.731201,-501.173492,
		-98.186081,421.766693,866.023560,1119.139404,1195.947998,1164.668945,1074.143921,954.892151,823.595642,
		687.585938,551.735596,416.947327,216.890198,-143.952423
};

float32_t validGuide[SIGNATURE_LEN] =

	{ -692.195801,-621.527222,-295.040314,-33.363327,41.884747,-3.340088,-47.429085,-147.653503,-393.280823,-820.853271,
	  -1342.676880,-1818.293335,-2154.334717,-2344.893799,-2417.472656,-2415.314941,-2355.524170,-2239.769775,-2046.067139,
	  -1787.682861,-1383.960815,-639.271790,453.514435,1533.826904,2173.278320,2344.927734,2277.119629,2124.105225,
	  1827.883179,1345.215576,721.850281,87.587135,-432.607971,-819.041077,-1162.653320,-1528.330688,-1870.080811,
	  -2056.945068,-2043.332275,-1869.897827,-1619.377197,-1331.252930,-1039.595703,-755.196411,-483.425873,
	  -229.990631,1.712646,255.925995,679.422729,1299.016724,1915.255249,2238.584229,2248.424561,2106.680176,1957.099121,
	  1806.061646,1657.113403,1515.399292,1381.105225,1248.908936,1119.573364,994.757324,876.854736,765.297729,659.573914,
	  559.407776,466.648315,380.626862,300.203369,224.535416,156.141922,96.116364,43.668369,-4.402928,-52.765228,
	  -152.015015,-385.989258,-799.316528,-1313.276611,-1790.295532,-2132.416260,-2330.987061,-2409.749756,
	  -2409.987061,-2350.574219,-2237.313232,-2048.604980,-1795.776001,-1403.910645,-681.009644,392.736328,
	  1477.541626,2143.118896,2340.789062,2285.523682,2139.234131,1853.001221,1380.252075,760.218628,121.173805,
	  -406.186096,-796.253601,-1138.051636,-1502.688477,-1849.592285,-2047.536987,-2047.580688,-1885.817017,-1640.805786,
	  -1349.830811,-1053.608643,-767.602478,-500.078064,-250.716125,-22.716637,226.111816,642.987183,1264.104736,
	  1895.328979,2236.744629,2251.237305,2101.467529,1945.527344,1798.207520,1656.182373,1518.021484,1381.161255,1246.758423 };
/*

{
	-719.992798,-639.515808,-333.960571,-83.120369,-16.418308,-68.968803,-112.072845,-146.702896,-171.282135,-191.162476,
	-209.851318,-230.383041,-247.597794,-258.674683,-286.765869,-394.576080,-611.368408,-873.400879,-1062.478882,
	-1123.342285,-1085.106079,-1000.882019,-896.695068,-783.974182,-669.742371,-558.165283,-453.515900,-355.991302,
	-179.367447,153.788116,586.304688,904.541626,1000.521606,939.272461,842.818970,698.216125,451.415314,119.571213,-185.050400,
	-372.447113,-439.214355,-436.847107,-400.941681,-349.602692,-296.496552,-245.553101,-197.805283,-147.998398,-97.606010,
	-49.659992,-11.088827,19.919292,45.263577,69.826569,93.124084,181.764221,416.114319,766.725952,1049.242676,1127.989624,
	1041.889160,911.265686,791.265625,678.931580,576.051453,480.758881,390.366913,302.777069,222.477112,151.212204,
	89.231987,32.891003,-19.158266,-67.622719,-109.683640,-144.369568,-173.525879,-199.524048,-221.307861,-236.795807,
	-246.417313,-252.541672,-280.268311,-387.758972,-603.931396,-865.587952,-1055.821777,-1118.642090,-1082.579590,-1000.043579,
	-897.160522,-784.125732,-668.474915,-555.358032,-450.411774,-353.879578,-184.432037,140.435013,571.608398,901.702332,
	1009.539795,952.971558,855.656738,715.709167,476.218933,148.629700,-159.904938,-355.478149,-430.153931,-432.890320,
	-400.060974,-348.411041,-292.657593,-239.074860,-191.038818,-145.721344,-101.876259,-56.503506,-12.871915,24.644819,
	52.683388,75.263992,96.018547,174.402252,395.849487,740.416565,1033.680420,1127.184448

};
*/
#endif /* INC_BWF_H_ */


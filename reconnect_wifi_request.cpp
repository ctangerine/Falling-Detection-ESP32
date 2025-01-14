// #include <Arduino.h>
// #include <vector>
// #include <string>
// #include <WiFi.h>
// #include <HTTPClient.h>
// #include <ArduinoJson.h>
// #include <NTPClient.h>
// #include <WiFiUdp.h>
// #include "support_vectors.h"
// #include "dual_coef.h"
// #include "mean.h"
// #include "scale.h"
// #include "train_value.h"
// #include "calculations.h"

// using namespace std;

// const int buzzerPin = 17;
// int skipFlag = 0;
// const char* ssid = "Thom Ho";
// const char* password = "12346789";

// // Define NTP Client to get time
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Update every minute

// // char record[150];
// // bool start = true;
// // std::vector<std::string> data_vector;
// unsigned long previousMillis = 0;
// const long interval = 1000; // 1 second

// // Store initial time
// time_t startEpochTime = 0;
// unsigned long startMillis = 0;

// const std::string data8 = R"(time;acc_x;acc_y;acc_z;gyro_x;gyro_y;gyro_z;acc_re;gyro_re
// 7112;6.5338;2.7485;4.5657;0.2699;-0.1715;-0.0133;8.43151298285189;0.32005491716266443
// 7121;6.529;2.9975;4.6376;0.3063;-0.1691;-0.0101;8.551033914679557;0.3500235849196451
// 7130;6.7253;3.2058;4.4556;0.3419;-0.1278;0.0111;8.680966829218967;0.36517346562969216
// 7142;6.62;2.9688;4.0582;0.2573;-0.1218;0.0;8.313071675379685;0.28467267167748994
// 7151;6.5649;2.988;4.1204;0.1607;-0.1348;-0.0345;8.306849713940899;0.21256947099713072
// 7164;6.6631;3.1388;4.1108;0.152;-0.1194;-0.0525;8.434906264446571;0.20029131284207013
// 7173;6.517;3.0263;3.8858;0.1171;-0.147;-0.0508;8.16879564746236;0.19468448833946683
// 7185;6.7133;2.9473;3.5602;0.0698;-0.1957;-0.0436;8.150460000515308;0.21230047103103658
// 7194;6.5912;2.636;3.2417;0.0224;-0.2249;-0.0454;7.803911348163817;0.2305275037820867
// 7203;6.4691;2.2386;2.8826;-0.047;-0.2525;-0.0548;7.427648856132066;0.262618144841517
// 7216;6.3135;1.9872;2.9449;-0.1015;-0.2643;-0.0665;7.244424207623405;0.29082467226836173
// 7225;6.268;1.7669;2.9856;-0.0986;-0.2771;-0.0446;7.1640468291322605;0.2974819826476891
// 7237;6.3758;1.2522;2.6025;-0.1575;-0.298;-0.0378;6.99941688499835;0.3391741293200294
// 7246;6.1914;0.8882;2.3966;-0.2086;-0.2906;-0.0438;6.698210564023798;0.36038973348307246
// 7259;6.6032;0.5866;1.9489;-0.2346;-0.2798;-0.0157;6.909743917830819;0.36547460924118924
// 7268;6.4907;0.1173;1.6544;-0.3082;-0.2751;-0.0216;6.699252580698835;0.4136832242187251
// 7280;6.2297;-0.2777;1.64;-0.384;-0.2543;-0.0085;6.447936055824375;0.46064817377256584
// 7289;6.2249;-0.6632;1.312;-0.362;-0.2517;0.0264;6.396136196955159;0.44169429473335964
// 7298;6.4643;-0.68;0.9768;-0.412;-0.2411;0.0436;6.5729531209343035;0.4793476504584121
// 7311;6.4021;-0.6632;1.0774;-0.384;-0.2575;0.0825;6.525910619216295;0.46964720801895543
// 7320;6.2704;-1.0415;0.7901;-0.3757;-0.249;0.1033;6.405224150644535;0.4624093208403135
// 7332;6.2369;-1.3408;0.9912;-0.4341;-0.2172;0.0826;6.455938637409745;0.4923833973642897
// 7341;6.177;-1.5155;1.2163;-0.4782;-0.1827;0.0357;6.475450172767913;0.5131559412108565
// 7410;5.5258;-4.3167;1.415;-0.7911;-0.3299;-0.1303;7.153362113719674;0.866978263856713
// 7419;5.8921;-4.3503;1.9201;-0.7047;-0.3631;-0.0743;7.571574242520507;0.796218682272653
// 7431;6.2991;-4.4484;2.3559;-0.5606;-0.3696;-0.0211;8.063323643510783;0.6718048302892737
// 7440;6.8594;-4.3191;2.5067;-0.4445;-0.3643;-0.0157;8.484664876116204;0.5749271519070916
// 7453;7.2233;-3.9959;2.7102;-0.314;-0.3295;-0.0404;8.688409735964346;0.4569446465382869
// 7462;7.5776;-3.4022;2.988;-0.222;-0.2955;-0.0738;8.827407920788525;0.3768961262735397
// 7471;8.0254;-2.4373;3.4979;-0.0885;-0.2573;-0.1183;9.087506856118459;0.29669922480518884
// 7483;8.2073;-1.5443;4.0247;0.0781;-0.1851;-0.1529;9.270536439171146;0.25246787914505087
// 7492;8.2624;-0.7087;4.2737;0.1555;-0.1364;-0.1847;9.32920259936507;0.27730723034208826
// 7505;8.5688;0.5219;4.1156;0.1791;-0.0584;-0.2834;9.52023510266422;0.34029829561724223
// 7514;8.4084;0.9601;3.9768;0.1243;-0.0278;-0.3802;9.350824605883698;0.4009680411204863
// 7526;8.1235;1.822;3.6679;-0.0498;-0.0438;-0.5145;9.0974956257203;0.5187569083877341
// 7535;7.1539;1.8004;4.0366;-0.2116;-0.1346;-0.5941;8.409153639338504;0.6448616363220873
// 7548;5.0949;1.1684;6.1866;-0.4634;-0.3793;-0.5914;8.099208858277455;0.8416424478363719
// 7557;4.1252;0.8021;8.339;-0.5769;-0.5693;-0.51;9.338070488596667;0.9576095759755121
// 7566;3.7014;0.3017;10.6015;-0.5633;-0.7422;-0.3957;11.233128998636133;1.0122974957985424
// 7578;4.7956;0.328;12.378;-0.2679;-0.8322;-0.2142;13.278563452422103;0.9001160425189633
// 7587;6.6343;0.8236;12.2463;0.081;-0.7783;-0.1211;13.95220832484951;0.791818855547151
// 7600;9.1985;2.976;10.0652;0.5641;-0.5664;-0.0136;13.956261293412359;0.7995015509678516
// 7609;11.0038;4.877;7.8338;0.7561;-0.4049;0.0632;14.36095978268862;0.8600148021981947
// 7621;11.7699;6.0406;4.9847;0.6755;-0.1945;0.1386;14.137419441326625;0.7164778154276655
// 7630;11.1498;5.4037;3.644;0.4258;-0.0512;0.1819;12.914981600064323;0.4658483551543356
// 7699;4.3574;-1.7214;1.9632;0.1408;0.1568;-0.0722;5.079793987948724;0.22276382112003734
// 7708;4.063;-1.0894;2.2841;0.3313;0.0286;-0.1375;4.786634952657242;0.35983871387053395
// 7717;4.0821;-0.3112;2.6384;0.5237;-0.1069;-0.1735;4.870476404829408;0.5619533343614931
// 7729;4.0893;1.2809;3.1484;0.7337;-0.2802;-0.1703;5.317471378390296;0.8036353775189342
// 7738;4.6112;2.8754;3.4548;0.8148;-0.3785;-0.122;6.439466875448619;0.9066671329655663
// 7751;5.612;4.8411;3.3854;0.7576;-0.423;-0.0472;8.148111828515855;0.8689733022366107
// 7760;6.9791;5.7868;3.4189;0.5648;-0.405;-0.0195;9.689363666412774;0.6952728169574875
// 7772;9.0549;6.1699;3.8355;0.1202;-0.2883;-0.0256;11.609045622703015;0.313401164643656
// 7781;10.1634;5.7269;4.3144;-0.2208;-0.17;-0.1047;12.438091916769226;0.2976822634958287
// 7790;10.0987;4.4939;4.8698;-0.496;-0.0298;-0.1888;12.078651867654767;0.5315538354673024
// 7803;9.3565;2.5666;5.6407;-0.6567;0.1295;-0.2065;11.222701292469653;0.7004765449320911
// 7812;8.6407;1.0774;5.8897;-0.5929;0.2124;-0.1673;10.5124237614358;0.6516390565335997
// 7824;7.4029;-0.3567;6.3135;-0.375;0.2793;-0.1009;9.736038493658496;0.47834537731643234
// 7833;6.4715;-1.063;6.2249;-0.144;0.2832;-0.0528;9.042104913127252;0.3220653349865521
// 7846;5.6455;-0.7063;5.7916;0.1619;0.2372;0.0388;8.118692043673045;0.28979456516642954
// 7855;5.1451;-0.0982;5.5354;0.32;0.1831;0.0849;7.557932945587702;0.3783300411016815
// 7867;5.114;1.0032;5.533;0.4482;0.1095;0.0953;7.600887792883145;0.4711216191176117
// 7876;5.1906;1.834;5.1858;0.46;0.044;0.0729;7.562962779228786;0.46781450383672374
// 7885;5.7557;2.6695;4.877;0.4096;-0.0005;0.0637;8.002464729069413;0.41452394381989566
// 7898;6.2489;3.0478;4.4029;0.2872;-0.038;0.0318;8.229420663691947;0.2914430990776759
// 7907;6.6774;3.1077;4.2186;0.1856;-0.0586;0.0149;8.487759186616925;0.19520074282645544
// 7919;6.9815;2.3391;4.0917;0.0608;-0.0644;0.0088;8.423463655171785;0.08900247187578557
// 7928;7.0653;1.8818;4.0103;0.0254;-0.0697;-0.0075;8.339193091660608;0.07456205469271887
// 7997;6.2968;2.0399;4.8315;0.0332;-0.2278;-0.0801;8.194771168251131;0.24374390248783662
// 8006;6.3877;2.2673;4.8674;0.0117;-0.2413;-0.0998;8.344755439196526;0.2613859598371726
// 8015;6.3805;2.1763;4.8483;-0.0145;-0.2467;-0.125;8.303798819215215;0.27694067956874807
// 8027;6.6798;2.3607;4.9464;-0.0552;-0.2577;-0.1572;8.640573215360194;0.30686832681135406
// 8036;6.5577;2.0111;4.8291;-0.0893;-0.265;-0.1853;8.38857313909821;0.33546323196439876
// 8049;6.9456;2.1859;4.9225;-0.1214;-0.2774;-0.2029;8.789227748784304;0.36449572013948256
// 8058;6.7971;1.8818;4.7453;-0.128;-0.2706;-0.1961;8.500565377667535;0.35785970714792686
// 8070;6.6942;1.5155;4.9057;-0.1079;-0.2621;-0.1829;8.436524543910247;0.3373295569617344
// 8079;6.7397;1.4269;4.7669;-0.0622;-0.2475;-0.1695;8.377525607839106;0.30635818905327145
// 8092;6.9504;1.4772;4.6304;0.0116;-0.2286;-0.1244;8.48120181106428;0.2605146445019934
// 8101;6.924;1.3408;4.6424;0.0694;-0.2222;-0.0897;8.443423381543768;0.2494700182386653
// 8110;7.0006;1.4748;4.5155;0.0931;-0.1986;-0.0492;8.460093123009935;0.22478925686073167)";

// void setup() {
//   Serial.begin(115200);
//   // pinMode(buzzerPin, OUTPUT);

//   // Connect to WiFi and fetch initial time
//   Serial.println("Connecting to WiFi...");
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWiFi connected");

//   // Initialize NTPClient and fetch initial time
//   timeClient.begin();
//   timeClient.update();
//   startEpochTime = timeClient.getEpochTime();
//   startMillis = millis();
//   Serial.println("Initial time fetched from NTP");

//   // Turn off WiFi
//   // WiFi.disconnect(true);
//   // WiFi.mode(WIFI_OFF);

//   Serial.println("WIFI - OFF");

//   // initializeMPU(); // Commented out
// }

// void loop() {
//   unsigned long currentMillis = millis();
//   if (skipFlag == 1) {
//     previousMillis = currentMillis;
//   }

//   // if (currentMillis - previousMillis >= interval) {
//   //   previousMillis = currentMillis;
//   Serial.println("Start calling");
//   sendDataToPredict();
  
// }

// void alarm(int duration) {
//   int beepDuration = duration / 8; // Calculate the duration of each beep
//   for (int i = 0; i < 8; i++) {
//     digitalWrite(buzzerPin, HIGH);
//     delay(beepDuration / 2);
//     digitalWrite(buzzerPin, LOW);
//     delay(beepDuration / 2);
//   }
//   skipFlag = 0;
// }

// void sendDataToPredict() {
//   // if (!data_vector.empty()) { // Commented out
//   //   data_vector.clear(); // Commented out

//     skipFlag = 0;
//     double value = predict(data8, support_vectors, dual_coef, intercept, gammma, mean, scale);
//     if (skipFlag == 0) {
//       skipFlag = 1;
//       callCloudFunction();
//       // alarm(10000); // Run alarm for 10s
//       Serial.println("Call ended");
//       delay(10000);
//       Serial.println("Start new call");
//       Serial.println("____________________________________________________________________________________");
//     }
//   // }
// }

// String getCurrentTime() {
//   unsigned long elapsedMillis = millis() - startMillis;
//   time_t currentTime = startEpochTime + elapsedMillis / 1000;

//   struct tm* timeInfo = localtime(&currentTime);
//   char timeString[40];
//   sprintf(timeString, "%04d-%02d-%02d %02d:%02d:%02d",
//           timeInfo->tm_year + 1900,
//           timeInfo->tm_mon + 1,
//           timeInfo->tm_mday,
//           timeInfo->tm_hour,
//           timeInfo->tm_min,
//           timeInfo->tm_sec);

//   return String(timeString);
// }

// void callCloudFunction() {
//   // Lấy thời gian hiện tại
//   String currentTime = getCurrentTime();
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, password);

//   // Ghi lại thời điểm bắt đầu
//   unsigned long startMillis = millis();

//   // Chờ kết nối WiFi
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//   }

//   unsigned long wifiConnectedMillis = millis();
//   Serial.print("WiFi connected in: ");
//   Serial.print(wifiConnectedMillis - startMillis);
//   Serial.println(" ms");

//   // Tạo HTTP client và gửi request
//   HTTPClient http;
//   http.begin("https://asia-southeast1-falling-detection-3e200.cloudfunctions.net/handleFallDetection");
//   http.addHeader("Content-Type", "application/json");

//   DynamicJsonDocument doc(256);
//   doc["deviceID"] = "DEVICE01";
//   doc["time"] = "time alkdf";

//   String requestBody;
//   serializeJson(doc, requestBody);

//   // Ghi lại thời điểm bắt đầu gửi request
//   unsigned long requestStartMillis = millis();

//   // Gửi POST request
//   int httpResponseCode = http.POST(requestBody);

//   // Ghi lại thời điểm nhận phản hồi
//   unsigned long responseMillis = millis();

//   // Log thời gian thực hiện
//   Serial.print("Time to send request: ");
//   Serial.print(requestStartMillis - wifiConnectedMillis);
//   Serial.println(" ms");

//   Serial.print("Time for server response: ");
//   Serial.print(responseMillis - requestStartMillis);
//   Serial.println(" ms");

//   Serial.print("Total time: ");
//   Serial.print(responseMillis - startMillis);
//   Serial.println(" ms");

//   if (httpResponseCode > 0) {
//     String response = http.getString();
//     // Serial.println("Server response: " + response);
//   } else {
//     Serial.print("Error sending POST: ");
//     Serial.println(httpResponseCode);
//   }

//   // Kết thúc HTTP
//   http.end();

//   // Ngắt kết nối WiFi
//   WiFi.disconnect(true);
//   WiFi.mode(WIFI_OFF);
// }

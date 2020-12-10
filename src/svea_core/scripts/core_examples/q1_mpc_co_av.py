#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import rospy
import numpy as np

from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray

from svea.svea_managers.mpc_path_following_sveas import SVEAMPC
from svea.controllers.mpc.mpc import MPC
from svea.controllers.mpc.parameters import parameters

from svea.track import Track

"""
__team__ = "Team 1"
__maintainers__ = "Roberto Castro Sundin, Astrid Lindstedt, Johan Hedin, Aravind Sadashiv, Sarthak Manocha”
__status__ = "Development"
"""

## SIMULATION PARAMS ##########################################################
# param_name = "simulation"
param_name = "ZOH-good"
params = parameters.get(param_name)

vehicle_name = ""
target_velocity = params.target_velocity# [m/s]
dt = params.dt # frequency of the model updates

# xs = [14.47, 37.06]
# ys = [1.60, 1.29]
# traj_x_init = np.linspace(xs[0], xs[1]).tolist()
# traj_y_init = np.linspace(ys[0], ys[1]).tolist()
# xs = [37.06,37.05]
# ys = [1.29,6.96]
# traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:])
# traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:])
# xs = [37.15,14.68]
# ys = [6.96,7.31]
# traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:])
# traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:])
# xs = [14.69,14.47]
# ys = [7.31,1.60]
# traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:-1])
# traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:-1])

# traj_x_init = [15.1, 15.399999999999993, 15.699999999999987, 15.99999999999998, 16.300000000000026, 16.600000000000072, 16.90000000000012, 17.200000000000166, 17.500000000000213, 17.80000000000026, 18.100000000000307, 18.400000000000354, 18.7000000000004, 19.000000000000448, 19.300000000000495, 19.60000000000054, 19.90000000000059, 20.200000000000635, 20.500000000000682, 20.80000000000073, 21.100000000000776, 21.400000000000823, 21.70000000000087, 22.000000000000917, 22.300000000000963, 22.60000000000101, 22.900000000001057, 23.200000000001104, 23.50000000000115, 23.800000000001198, 24.100000000001245, 24.40000000000129, 24.70000000000134, 25.000000000001386, 25.300000000001432, 25.60000000000148, 25.900000000001526, 26.200000000001573, 26.50000000000162, 26.800000000001667, 27.100000000001714, 27.40000000000176, 27.700000000001808, 28.000000000001855, 28.3000000000019, 28.60000000000195, 28.900000000001995, 29.200000000002042, 29.50000000000209, 29.800000000002136, 30.100000000002183, 30.40000000000223, 30.700000000002277, 31.000000000002323, 31.30000000000237, 31.600000000002417, 31.900000000002464, 32.20000000000244, 32.50000000000238, 32.80000000000232, 33.10000000000226, 33.4000000000022, 33.70000000000214, 33.99943366737961, 34.2988077805138, 34.59880778051374, 34.89824144789121, 35.197615561025394, 35.497615561025334, 35.89704922840281, 36.19623533953543, 36.48506204754969, 36.74532344436383, 36.96061420825326, 37.1173636972051, 37.20569136101907, 37.22002955209248, 37.21123133977523, 37.21935112063529, 37.227470901495344, 37.2355906823554, 37.24371046321546, 37.251830244075514, 37.25995002493557, 37.268069805795626, 37.27618958665568, 37.28430936751574, 37.292429148375795, 37.27228585758349, 37.17071979433642, 37.002252856613275, 36.77750419249801, 36.51064061346214, 36.21848360330716, 35.91881306473572, 35.618883726958735, 35.31895438918175, 35.01902505140476, 34.71909571362777, 34.419166375850786, 34.1192370380738, 33.81930770029681, 33.519378362519824, 33.219449024742836, 32.91951968696585, 32.61959034918886, 32.319661011411874, 32.01973167363489, 31.719802335857796, 31.419872998080702, 31.11994366030361, 30.820014322526514, 30.52008498474942, 30.220155646972326, 29.920226309195233, 29.62029697141814, 29.320367633641045, 29.02043829586395, 28.720508958086857, 28.420579620309763, 28.12065028253267, 27.820720944755575, 27.52079160697848, 27.220862269201387, 26.920932931424293, 26.6210035936472, 26.321074255870105, 26.02114491809301, 25.721215580315917, 25.4215091523501, 25.12183861377865, 24.821909276001556, 24.52220284803574, 24.222532309464288, 23.922602971687194, 23.622896543721378, 23.323226005149927, 23.023296667372833, 22.723590239407017, 22.423919700835565, 22.12399036305847, 21.824061025281377, 21.52435459731556, 21.22468405874411, 20.924754720967016, 20.6250482930012, 20.32537775442975, 20.025448416652655, 19.72574198868684, 19.426071450115387, 19.126142112338293, 18.826435684372477, 18.526765145801026, 18.22683580802393, 17.927129380058116, 17.627458841486664, 17.32752950370957, 17.027823075743754, 16.728152537172303, 16.42822319939521, 16.128516771429393, 15.82884623285794, 15.528916895080899, 14.930175856249761, 14.643112997893926, 14.386116617597159, 14.1753862299722, 14.024205018189717, 13.942102541563385, 13.934254049131011, 14.001154262910482, 14.07049919607521, 14.13739940985468, 14.206744343019407, 14.273644556798878, 14.342989489963605, 14.409889703743076, 14.479234636907803, 14.546134850687274, 14.615479783852, 14.682379997631472, 14.751724930796199, 14.798490628573454, 14.861913744272584, 14.925336859971715]
# traj_y_init = [1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.524182495318074, 1.5072752483923575, 1.5072752483923575, 1.4914577437104315, 1.474550496784715, 1.474550496784715, 1.458732992102789, 1.4624665427491146, 1.540603609706744, 1.6882188996446383, 1.8960076276329458, 2.1508720351292245, 2.436746994333875, 2.7356126577544804, 3.0353347057814974, 3.3352248009143373, 3.635114896047177, 3.935004991180017, 4.234895086312857, 4.534785181445697, 4.834675276578537, 5.134565371711377, 5.434455466844216, 5.734345561977056, 6.034235657109896, 6.631997313953263, 6.9134410974902, 7.160716570170533, 7.358236965252915, 7.493551778482166, 7.55813157340593, 7.568537481621252, 7.562026541550107, 7.555515601478962, 7.5490046614078175, 7.5424937213366725, 7.535982781265528, 7.529471841194383, 7.522960901123238, 7.516449961052093, 7.509939020980948, 7.503428080909803, 7.496917140838658, 7.490406200767513, 7.483895260696368, 7.477384320625223, 7.470873380554078, 7.464362440482933, 7.457851500411788, 7.451340560340643, 7.444829620269498, 7.438318680198353, 7.4318077401272085, 7.4252968000560635, 7.418785859984919, 7.412274919913774, 7.405763979842629, 7.399253039771484, 7.392742099700339, 7.386231159629194, 7.379720219558049, 7.373209279486904, 7.366698339415759, 7.360187399344614, 7.353676459273469, 7.347165519202324, 7.356480649338857, 7.366886557554179, 7.360375617483034, 7.3696907476195666, 7.380096655834889, 7.373585715763744, 7.382900845900276, 7.393306754115598, 7.386795814044453, 7.396110944180986, 7.406516852396308, 7.400005912325163, 7.393494972254018, 7.402810102390551, 7.413216010605873, 7.406705070534728, 7.4160202006712606, 7.426426108886583, 7.419915168815438, 7.42923029895197, 7.439636207167292, 7.433125267096147, 7.44244039723268, 7.452846305448002, 7.446335365376857, 7.45565049551339, 7.466056403728712, 7.459545463657567, 7.4688605937940995, 7.479266502009422, 7.472755561938277, 7.482070692074809, 7.492476600290131, 7.485965660218986, 7.485054842980147, 7.400667735862582, 7.247438727481479, 7.03502645799939, 6.77682012635922, 6.489095515269175, 6.189989063212997, 5.898354651849457, 5.607291860102596, 5.315657448739056, 5.024594656992195, 4.7329602456286555, 4.441897453881794, 4.1502630425182545, 3.8592002507713956, 3.5675658394078584, 3.276503047661, 2.984868636297463, 2.6938058445506043, 2.397624001547195, 2.1044047859507184, 1.8111855703542377]

# traj_x_init = [1.64245519603, 2.1376819888, 2.62307066076, 3.0021437254, 3.43173705571, 3.8771816939, 4.36574780376, 4.91093442552, 5.45123738705, 5.96731156168, 6.47705256374, 6.93243341669, 7.30568562413, 7.67980080042, 8.05765974495, 8.51170555776, 9.02108774552, 9.53429978406, 10.0872936795, 10.6427474546, 11.1713915976, 11.7355328498, 12.2906121427, 12.8592856099, 13.4579548622, 14.0352001983, 14.6264776292, 15.1706923957, 15.741496675, 16.2786230881, 16.8385169306, 17.3898492338, 17.9637512959, 18.4685885484, 18.9318715442, 19.2992775906, 19.5489627993, 19.7550668534, 19.8922828325, 19.9358857617, 19.875740498, 19.7128672934, 19.4182917857, 19.0067354734, 18.5582562634, 18.0619505168, 17.550565999, 17.0117144268, 16.4313689415, 15.8469623972, 15.2764334061, 14.7168366524, 14.1518117803, 13.5802310649, 13.0124560335, 12.4505138476, 11.890769634, 11.3024126797, 10.7633606326, 10.1916734518, 9.65363812175, 9.04529260622, 8.48913669257, 7.93169146168, 7.35742478966, 6.81457454901, 6.32027540648, 5.8691386872, 5.4361972332, 5.09124761713, 4.84377110105, 4.5323705373, 4.15947609215, 3.69150854321, 3.19371409334, 2.72078385919, 2.37265120619, 2.00599342426, 1.71820771129, 1.34837947971, 0.932305087854, 0.442249875753, -0.0552090081656, -0.602397639444, -1.11421423267, -1.67780563055, -2.24637663572, -2.80966851137, -3.3658973937, -3.91110940777, -4.47709223041, -5.05750435938, -5.64351560462, -6.21773816011, -6.76686930875, -7.32077872474, -7.88747524546, -8.46711025225, -9.04964472026, -9.65586154065, -10.1917283609, -10.7310629261, -11.2457160497, -11.7873938475, -12.3174421101, -12.7789648654, -13.1639729092, -13.4705888096, -13.7619936967, -13.9971000876, -14.1574723831, -14.2595513475, -14.3099658152, -14.2245339326, -13.9610034229, -13.5903769534, -13.1232190284, -12.62399368, -12.11353961, -11.5817582839, -11.0334760569, -10.5014687844, -9.96280248157, -9.42880830088, -8.89829980724, -8.46413184862, -8.20119042248, -8.03244384746, -7.98354620412, -7.90481107292, -7.69166898346, -7.33616603667, -6.91064541434, -6.54213489192, -6.25742083794, -6.05376238239, -5.73710003413, -5.44021301829, -5.33300323135, -5.43832008959, -5.35458031217, -5.13607700343, -4.7051993444, -4.16916072803, -3.62260894065, -3.0181593912, -2.48431346889, -2.02548650052, -1.52611553431, -1.04968147408, -0.620497036031, -0.36239118339, -0.239304272686, -0.184119500922, -0.145405625812, -0.129901624798, -0.12533771643, -0.122613317304, -0.121628573493, -0.121403458904, -0.121030249393, -0.120827016837]
# traj_y_init = [0.346267964861, 0.631706884508, 0.965066607292, 1.35351802925, 1.73565106016, 2.09834502735, 2.37478749843, 2.50544809619, 2.48103522269, 2.28376285048, 2.04170910568, 1.71577380111, 1.33764378757, 0.941089562536, 0.56277486913, 0.214535581259, -0.107048886095, -0.358856817645, -0.596759492273, -0.738173276883, -0.830919565033, -0.926258082894, -0.974769083989, -0.999951343594, -1.01950429551, -1.04309466956, -1.07437620178, -1.15612805021, -1.24459056571, -1.34901673559, -1.422728819, -1.45997439665, -1.40211241427, -1.21710073642, -0.917537280199, -0.526216728888, -0.0771471298587, 0.393428286331, 0.925766308221, 1.49501575478, 2.05080930134, 2.5931162645, 3.0634553726, 3.46767615812, 3.76202028321, 3.99301347424, 4.12377191712, 4.20706239633, 4.23160622803, 4.23410944782, 4.23320230555, 4.26639854607, 4.28501479116, 4.2713972948, 4.2272021159, 4.18017036666, 4.15073444162, 4.16932436265, 4.20843551725, 4.23876636577, 4.2875258918, 4.32076173776, 4.33542243274, 4.36200815755, 4.42809088954, 4.57749028971, 4.77926174837, 5.05798022851, 5.38157326118, 5.79502502712, 6.30402369634, 6.72831391168, 7.05678445574, 7.28871290784, 7.33236373314, 7.15688372126, 6.85952037061, 6.50406101569, 6.08664281737, 5.75176507126, 5.55801494361, 5.44692521703, 5.3790736026, 5.36204094961, 5.39010462155, 5.387058755, 5.38418296317, 5.400249022, 5.40256485746, 5.46526786606, 5.51679535792, 5.58599459199, 5.63888289596, 5.6707185334, 5.73868340939, 5.77429510313, 5.79240619334, 5.78202320361, 5.8082301814, 5.9021344406, 5.96291519043, 6.09045764872, 6.19178749058, 6.20951785746, 6.10395721251, 5.85594563825, 5.50138073271, 5.11255563589, 4.64102795547, 4.14836574614, 3.61269988121, 3.04388799772, 2.48501506608, 1.97497770817, 1.52704104583, 1.13795013172, 0.873126551842, 0.732348968445, 0.621296933562, 0.520687468413, 0.460607350724, 0.43532313332, 0.537223288423, 0.667989688878, 0.822985832618, 1.08655335418, 1.45901466449, 1.94343358798, 2.5184611309, 3.04701732225, 3.41650957694, 3.56941488868, 3.59991275125, 3.46511523276, 3.1274175484, 2.74443027671, 2.36725115645, 1.97109931038, 1.4976993638, 1.04936033107, 0.660364776358, 0.271598304985, -0.0218492924488, -0.151648580287, -0.152779212038, -0.194571955576, -0.201702438159, -0.141118148484, -0.122224304355, -0.107333026778, -0.0630868484576, -0.0610633790103, -0.0501662350875, -0.0426029510993, -0.0394217910232, -0.0345780098697, -0.029989948684, -0.0238043245372, -0.0193714228983, -0.0174885166636, -0.0143587409777, -0.0125134659469]

# traj_x_init = traj_x_init[0:16]
# traj_y_init = traj_y_init[0:16]

traj_x_init =  [0.0, 0.3970152951844882, 0.7949160416307335, 1.194882709004963, 1.5938927281820854, 1.969158200955605, 2.2769649823962363, 2.5363254108050395, 2.796523521527946, 3.056011354938678, 3.325775041783727, 3.6469844544421632, 4.030080667110768, 4.4300633544135675, 4.8299219779746725, 5.229835865376676, 5.62891627739474, 5.997892458710568, 6.331456415803458, 6.663748136986109, 6.995825428200018, 7.327562225164367, 7.657595790373666, 7.981356869154896, 8.314776013403305, 8.640700501115138, 9.004946772138064, 9.40001093290932, 9.795959189914505, 10.191912764227464, 10.587854794611438, 10.98380573478521, 11.37968780470507, 11.775748828979744, 12.16797998867662, 12.562055570496122, 12.955538121298778, 13.348285431943975, 13.743039353244956, 14.134808032386275, 14.530199530683765, 14.92408124280592, 15.32346058187343, 15.72348680997623, 16.123287881615987, 16.522604967938513, 16.92253065163866, 17.32185325289521, 17.72167208863961, 18.120738710252155, 18.498880938350716, 18.813110813338735, 19.028114083915522, 19.13147598436156, 19.242223812585937, 19.39275530148327, 19.447668399643117, 19.45987407444446, 19.47605323744688, 19.491108145479128, 19.462028024297542, 19.303386387996536, 19.032696658696285, 18.68016731633384, 18.28604914730243, 17.88811146649408, 17.490042211895044, 17.09206925861236, 16.69404211474242, 16.29599038470714, 15.89801693297977, 15.499990279226315, 15.10193707906526, 14.703947979548284, 14.30605227406383, 13.906817030639028, 13.50878929478461, 13.110002275377736, 12.711440671764118, 12.313230111056255, 11.914129134919566, 11.516325483424032, 11.116931126361695, 10.719190319600186, 10.319921355872314, 9.921899662049046, 9.523104255258689, 9.12455391272167, 8.726392799699308, 8.326589329897624, 7.928134623147803, 7.529878472650348, 7.1310775952873175, 6.731166991635749, 6.341115471815019, 5.998172931725923, 5.713839836885226, 5.43523061251047, 5.156723807922349, 4.878063580011592, 4.5993254315250045, 4.321217214776714, 4.018737079340099, 3.650335349979457, 3.252016148763709, 2.8681095033752513, 2.541058727555579, 2.255118830465906, 1.9657902904372548, 1.6775510434634133, 1.3883419032919724, 1.1028686455971688, 0.7847313511524866, 0.4060603882005029, 0.0071166294605670725, -0.38964085189955405, -0.7886597579398823, -1.1873933611381542, -1.5861658851568903, -1.9849403977888203, -2.3837134397484654, -2.7824866823362324, -3.1812599351748485, -3.580033180432478, -3.978806426724301, -4.377579673068965, -4.776352919374556, -5.175126165685469, -5.5738994119966705, -5.972672658307641, -6.371445904618267, -6.77021915093194, -7.168992397240012, -7.567765643475923, -7.966538890301158, -8.365312136041563, -8.764085367782766, -9.16285871385019, -9.561631849465954, -9.960402269306435, -10.359194867846423, -10.757946639848843, -11.156117701391501, -11.555044707054899, -11.953836054842398, -12.334413837278035, -12.706838957343951, -13.079609358514793, -13.451206548226724, -13.807539022524097, -14.097985388580527, -14.281258072465556, -14.33471438102138, -14.295478307110116, -14.250575190772286, -14.205304308966785, -14.16024125991543, -14.115562047947948, -14.064809644962558, -13.978289799144298, -13.775360968025941, -13.470861997376117, -13.097979637312596, -12.701472310811813, -12.303070652726626, -11.904691673387974, -11.506310062655778, -11.107928083188135, -10.70954637017629, -10.311168263036087, -9.912764730158672, -9.5143901943059, -9.114566119686819, -8.729381188278932, -8.40034455526727, -8.161151314323421, -8.002434375928853, -7.961129042852188, -7.983827890733702, -7.949803192838047, -7.804613092280166, -7.543733552921224, -7.197609966152073, -6.805297099963484, -6.411816274024558, -6.043157335282016, -5.688404403725891, -5.408186907675219, -5.23616527430911, -5.194470709946565, -5.1985411827092225, -5.19746221396982, -5.14097430774665, -4.96123534970724, -4.674326290201955, -4.3116384158983845, -3.925728343927682, -3.53728817589405, -3.145458538087468, -2.748767902125341, -2.3495897548179583, -1.951513156296159, -1.5517573297530978, -1.1529377275285135, -0.7536273061463838]
traj_y_init=  [0.0, 0.04885191113786227, 0.08793344847635556, 0.08756183814527088, 0.09318849247558171, 0.23191634240178693, 0.4856492320367536, 0.7901524958695099, 1.09397587548, 1.398397752970759, 1.6941009585943463, 1.929500294061544, 2.043351504061431, 2.0371335629414764, 2.032447088117864, 2.0241540580837336, 2.007267345542541, 1.8552824010197724, 1.634234683008355, 1.4115661678695701, 1.18858033171721, 0.9650934293578216, 0.7392607022449988, 0.50458230077913, 0.2834660675433487, 0.05107863734759109, -0.10976316621062064, -0.1744643878282318, -0.23121779135703577, -0.2878918182003723, -0.34473945830695846, -0.4015175709822078, -0.4587010313086076, -0.5148311259277646, -0.5933782922029834, -0.6609922396271929, -0.7322708005833045, -0.8076214525685157, -0.8714715340794182, -0.952556866410943, -1.0124182127220547, -1.0834493816098982, -1.0954899693465616, -1.0957802405355, -1.087471312681097, -1.0629465985756255, -1.0601646683045522, -1.034811432473039, -1.0295394343079, -1.0043948021672133, -0.8770009786678163, -0.6307210898888014, -0.29458766172353396, 0.09163758700749221, 0.4753618619092355, 0.8458534631970431, 1.2414041060308378, 1.641282477555838, 2.0408196678427077, 2.4422958769371568, 2.840924737951086, 3.207079822820284, 3.499945391464769, 3.687739351188746, 3.7546157517253906, 3.793705485858321, 3.8332479990622144, 3.873456885181261, 3.9131213280247406, 3.952548787745082, 3.99275785980571, 4.032437572277192, 4.071843204060905, 4.111831536070871, 4.153360600681758, 4.176196977371035, 4.216017375055643, 4.244934387932348, 4.2774459702455685, 4.314304825200973, 4.339311994481893, 4.381892302930688, 4.402708281148841, 4.446330433656247, 4.468711455554707, 4.50861381005127, 4.5374148544212325, 4.5700835654413465, 4.607644499481222, 4.62274599268521, 4.656731995707999, 4.693301844139287, 4.71968486663297, 4.756194365577003, 4.838434773840251, 5.042457161965224, 5.323860791582111, 5.610872849211906, 5.897989281531854, 6.18494160186868, 6.471647630926343, 6.761268816842035, 7.022170790512122, 7.17583578245679, 7.199267820271638, 7.089815234672386, 6.861027141625332, 6.581146963229546, 6.304963834938995, 6.027581282533144, 5.751552778226652, 5.468707462639155, 5.229147510994295, 5.10074169397195, 5.106238176003025, 5.156142438999902, 5.185588642933584, 5.217191656734626, 5.248500439994466, 5.279794189893543, 5.311099057664671, 5.342402408743223, 5.3737056823281675, 5.405009013223252, 5.436312336300127, 5.467615658977537, 5.49891898195037, 5.53022230488291, 5.561525627813311, 5.592828950745384, 5.624132273679161, 5.655435596597303, 5.686738919544216, 5.71804224286231, 5.7493455631491, 5.780648889015917, 5.8119522868902145, 5.843255096706918, 5.874558989019712, 5.905876850412613, 5.937080631663292, 5.968494411575621, 6.003130929718834, 6.059031585858515, 6.058927446158728, 5.935702529945456, 5.79054285620842, 5.645367666108261, 5.497289581901186, 5.315041741747, 5.041861250487358, 4.687097829497458, 4.291912445496892, 3.8937469434307252, 3.4962696877092476, 3.09884528381092, 2.701390608475437, 2.3038669423448073, 1.9072751143743705, 1.51684829021397, 1.1730189335287384, 0.9150303623309298, 0.7725084504934789, 0.7201595467258669, 0.6848560547675818, 0.6488224018322029, 0.612873937108676, 0.576934070512259, 0.5409893609552663, 0.5050189732245739, 0.46923208435950625, 0.4332360792713855, 0.42916170062266007, 0.5328386798004362, 0.759033602632514, 1.0789499517551147, 1.4460910611250668, 1.8427905849310355, 2.2419696027317757, 2.640755182994864, 3.0121083745128043, 3.3141658899156483, 3.513206460753693, 3.586730864693, 3.5241674335812108, 3.368077223133574, 3.1836106822171035, 2.8993658190928993, 2.5388741230387413, 2.1419318941499816, 1.7419960924546116, 1.3419795832992354, 0.9464680755973951, 0.5900630515422984, 0.31233803468596977, 0.14563470151084637, 0.04042737335473797, -0.05463838156357541, -0.13505840090915053, -0.179496952044338, -0.14820682198747556, -0.10972792756661992, -0.09463826338291201, -0.06608033887514082, -0.0443093808263668]

###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################

class Node:
    def __init__(self):
        rospy.init_node('floor2_mpc_co_av')
        initial_traj_pub = rospy.Publisher('init_traj', Float32MultiArray, queue_size=1, latch=True)
        initial_traj_pub.publish(Float32MultiArray(data=traj_x_init+traj_y_init)) # TODO: This assumes traj_x_init are python arrays

        # grab parameters from launch-file
        start_pt_param = rospy.search_param('start_pt')
        is_sim_param = rospy.search_param('is_sim')
        use_rviz_param = rospy.search_param('use_rviz')
        use_matplotlib_param = rospy.search_param('use_matplotlib')
        run_lidar_param = rospy.search_param('run_lidar')

        start_pt = rospy.get_param(start_pt_param, default_init_pt)
        if isinstance(start_pt, str):
            start_pt = start_pt.split(',')
            start_pt = [float(curr) for curr in start_pt]
            start_pt = VehicleState(*start_pt)

        self.is_sim = rospy.get_param(is_sim_param, True)
        self.use_rviz = rospy.get_param(use_rviz_param, False)
        self.use_matplotlib = rospy.get_param(use_matplotlib_param, False)
        self.run_lidar = rospy.get_param(run_lidar_param, True)

        #This subscriber and its callback function is the local planner
        traj_upd_sub = rospy.Subscriber('trajectory_updates', Path, self.callback_traj)
        collision_sub = rospy.Subscriber('collision', Bool, self.callback_collision)
        self.coll_t = None
        # select data handler based on the ros params
        if self.use_rviz:
            self.DataHandler = RVIZPathHandler
        elif self.use_matplotlib:
            self.DataHandler = TrajDataHandler
        else:
            # DataHandler = BasicDataHandler
            self.DataHandler = RVIZPathHandler

        if self.is_sim:
            # start the simulation
            model_for_sim = SimpleBicycleModel(start_pt)
            self.simulator = SimSVEA(vehicle_name, model_for_sim,
                                dt=dt, start_paused=True, run_lidar=self.run_lidar).start()

        self.collision = False

        self.traj_x = traj_x_init
        self.traj_y = traj_y_init

    def callback_traj(self,path):
        self.traj_x = [i.pose.position.x for i in path.poses]
        self.traj_y = [i.pose.position.y for i in path.poses]
        self.svea.update_traj(self.traj_x, self.traj_y)
        # if MPC.. this calculates speed profile etc.
        # self.svea.update_traj_mpc(self.traj_x, self.traj_y)

    def callback_collision(self, data):
        # print(data.data)
        self.collision = data.data

    def run(self):
        self.svea = SVEAMPC(
            vehicle_name,
            LocalizationInterface,
            MPC,
            self.traj_x, self.traj_y,
            data_handler = self.DataHandler,
            target_velocity=params.target_velocity,
            dl = dt*params.low_lim,
            low_lim = params.low_lim,
        )

        ulb = [-1e2,-np.deg2rad(40)]
        uub = [ 1e2, np.deg2rad(40)]
        xlb = [-np.inf]*3+[-1]
        xub = [ np.inf]*3+[3.6]

        # To use this again, switch from update_traj to update_traj_mpc
        self.svea.controller.build_solver(
            dt,
            Q=params.Q,
            R=params.R,
            P=params.P,
            ulb=ulb,
            uub=uub,
            xlb=xlb,
            xub=xub,
            max_cpu_time=0.8*dt,
            horizon=params.horizon,
            model_type=params.model_type,
            solver_=params.solver_,
            TAU = params.TAU,
            N_IND_SEARCH = params.N_IND_SEARCH,
        )
        # self.svea.update_traj_mpc(self.traj_x, self.traj_y)
        self.svea.update_traj(self.traj_x, self.traj_y)
        self.svea.start(wait=True)

        track = Track(vehicle_name, publish_track=True)
        track.start()

        if self.is_sim:
            # start simulation
            self.simulator.toggle_pause_simulation()

        # simulation loop
        self.svea.controller.target_velocity = target_velocity
        self.svea.pid.target_velocity = target_velocity
        self.svea.pid.k = 0.5  # look forward gain
        self.svea.pid.Lfc = 0.7 # look-ahead distance
        self.svea.pid.K_p = 1.2  # speed control propotional gain
        self.svea.pid.K_i = 0  # speed control integral gain
        self.svea.pid.K_d = 0.0  # speed control derivitive gain
        self.svea.pid.L = 0.324  # [m] wheel base of vehicle
        while not self.svea.is_finished and not rospy.is_shutdown():
            state = self.svea.wait_for_state()

            #This step updates the global path
            # svea.update_traj(self.traj_x, self.traj_y)

            # compute control input
            if self.collision:
                # if self.coll_t == None:
                #     self.coll_t = rospy.Time.now()
                # deltaT = rospy.Time.now()
                # print(deltaT)
                # if deltaT>rospy.Duration(5):
                # steering, velocity = self.svea.compute_control()
                # steering,_  = self.svea.compute_pid_control()
                # steering, velocity = self.svea.compute_pid_control()
                self.svea.send_control(0, 0)
            else:
                # steering, velocity = self.svea.compute_control()
                steering, velocity = self.svea.compute_pid_control()
                self.svea.send_control(steering, velocity)

            # visualize data
            if self.use_matplotlib or self.use_rviz:
                self.svea.visualize_data()
            else:
                self.svea.visualize_data()
                # rospy.loginfo_throttle(1, state)

        if not rospy.is_shutdown():
            rospy.loginfo("Trajectory finished!")

        rospy.spin()

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass

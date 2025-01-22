/* 该例子解决下面的凸二次规划问题：
	minimize x + y + x^2 + x*y + y^2 + y*z + z^2
	subject to x + 2 y + 3 z >= 4
	x + y >= 1
	x, y, z 非负

使用预定义的参数矩阵A和Q进行求解（一般在参数已存入外部数据时使用），否则不推荐使用稠密
矩阵的方法。
*/

#include "gurobi_c++.h"
#include <chrono>
using namespace std;

int main(int argc, char* argv[])
{
	// CPU 计时
    auto cpu_start = std::chrono::high_resolution_clock::now();

	GRBEnv* env = 0;
	cout<<GRB_INFINITY<<endl;
	try {
		env = new GRBEnv();
		GRBModel model = GRBModel(* env);

		// Create Variables
		GRBVar x = model.addVar(0, 100.0, 1.0, GRB_CONTINUOUS, "x");
		GRBVar y = model.addVar(0, 100.0, 1.0, GRB_CONTINUOUS, "y");
		GRBVar z = model.addVar(0, 100.0, 1.0, GRB_CONTINUOUS, "z");

		model.setObjective(x + y + x * x + x * y + y * y + y * z + z * z, GRB_MINIMIZE);
		
		model.addConstr(x + 2 * y + 3 * z >= 4, "c0");
		model.addConstr(x +  y >= 1, "c1");

		// Optimize model
		model.optimize();
		cout << x.get(GRB_StringAttr_VarName) << " "
			<< x.get(GRB_DoubleAttr_X) << endl;
		cout << y.get(GRB_StringAttr_VarName) << " "
			<< y.get(GRB_DoubleAttr_X) << endl;
		cout << z.get(GRB_StringAttr_VarName) << " "
			<< z.get(GRB_DoubleAttr_X) << endl;
		cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

	}
	catch (GRBException e) {
		cout << " Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (...) {
		cout << " Exception during optimization " << endl;
	}
	delete env;
	// CPU 计时结束
    auto cpu_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> cpu_duration = cpu_end - cpu_start;
    std::cout << "CPU time: " << cpu_duration.count() << " ms" << std::endl;
	return 0;
}
#include <iostream>
#include <unordered_map>
#include <map>
#include <vector>
#include <tuple>
#include <utility>  // for std::pair
#include <algorithm> // for std::swap

class QuadExpr {
private:
    // 线性项系数: var_index -> coefficient
    std::unordered_map<int, double> linear_terms;
    
    // 二次项系数: (var_index1, var_index2) -> coefficient
    // 使用pair作为key来存储两个变量的索引
    std::map<std::pair<int, int>, double> quad_terms;
    
    // 常数项
    double constant;

public:
    QuadExpr() : constant(0.0) {}
    
    // 添加线性项：coefficient * var
    void addTerm(double coeff, int var_index) {
        linear_terms[var_index] += coeff;
    }
    
    // 添加二次项：coefficient * var1 * var2
    void addTerm(double coeff, int var1_index, int var2_index) {
        // 确保 var1_index <= var2_index，保持一致性
        if (var1_index > var2_index) {
            std::swap(var1_index, var2_index);
        }
        quad_terms[{var1_index, var2_index}] += coeff;
    }
    
    // 添加常数项
    void addConstant(double value) {
        constant += value;
    }
    
    // 获取线性项系数
    double getCoeff(int var_index) const {
        auto it = linear_terms.find(var_index);
        return (it != linear_terms.end()) ? it->second : 0.0;
    }
    
    // 获取二次项系数
    double getQuadCoeff(int var1_index, int var2_index) const {
        if (var1_index > var2_index) {
            std::swap(var1_index, var2_index);
        }
        auto it = quad_terms.find({var1_index, var2_index});
        return (it != quad_terms.end()) ? it->second : 0.0;
    }
    
    // 运算符重载
    QuadExpr operator+(const QuadExpr& other) const {
        QuadExpr result(*this);
        
        // 合并线性项
        for (const auto& term : other.linear_terms) {
            result.addTerm(term.second, term.first);
        }
        
        // 合并二次项
        for (const auto& term : other.quad_terms) {
            result.addTerm(term.second, term.first.first, term.first.second);
        }
        
        result.constant += other.constant;
        return result;
    }
    
    // 乘以标量
    QuadExpr operator*(double scalar) const {
        QuadExpr result;
        
        // 缩放线性项
        for (const auto& term : linear_terms) {
            result.addTerm(term.second * scalar, term.first);
        }
        
        // 缩放二次项
        for (const auto& term : quad_terms) {
            result.addTerm(term.second * scalar, 
                          term.first.first, term.first.second);
        }
        
        result.constant = constant * scalar;
        return result;
    }
    
    // 获取所有非零项
    void getTerms(std::vector<std::tuple<double, int>>& linear_coeffs,
                 std::vector<std::tuple<double, int, int>>& quad_coeffs) const {
        // 收集线性项
        for (const auto& term : linear_terms) {
            if (term.second != 0.0) {
                linear_coeffs.emplace_back(term.second, term.first);
            }
        }
        
        // 收集二次项
        for (const auto& term : quad_terms) {
            if (term.second != 0.0) {
                quad_coeffs.emplace_back(term.second, 
                                       term.first.first, 
                                       term.first.second);
            }
        }
    }
};

// 使用示例
void example() {
    QuadExpr expr;
    
    // 添加项: 2x + 3y + 4x*y + 5x^2 + 1
    expr.addTerm(2.0, 0);        // 2x
    expr.addTerm(3.0, 1);        // 3y
    expr.addTerm(4.0, 0, 1);     // 4x*y
    expr.addTerm(5.0, 0, 0);     // 5x^2
    expr.addConstant(1.0);       // +1
    
    // 获取系数
    double x_coeff = expr.getCoeff(0);          // 线性项x的系数
    double xy_coeff = expr.getQuadCoeff(0, 1);  // x*y的系数
    
    // 获取所有项
    std::vector<std::tuple<double, int>> linear_terms;
    std::vector<std::tuple<double, int, int>> quad_terms;
    expr.getTerms(linear_terms, quad_terms);
}


int main() {
    QuadExpr expr;
    
    // 添加项: 2x + 3y + 4x*y + 5x^2 + 1
    expr.addTerm(2.0, 0);        // 2x
    expr.addTerm(3.0, 1);        // 3y
    expr.addTerm(4.0, 0, 1);     // 4x*y
    expr.addTerm(5.0, 0, 0);     // 5x^2
    expr.addConstant(1.0);       // +1
    
    // 获取系数并打印
    std::cout << "x coefficient: " << expr.getCoeff(0) << std::endl;
    std::cout << "y coefficient: " << expr.getCoeff(1) << std::endl;
    std::cout << "x*y coefficient: " << expr.getQuadCoeff(0, 1) << std::endl;
    std::cout << "x^2 coefficient: " << expr.getQuadCoeff(0, 0) << std::endl;
    
    // 获取所有项
    std::vector<std::tuple<double, int>> linear_terms;
    std::vector<std::tuple<double, int, int>> quad_terms;
    expr.getTerms(linear_terms, quad_terms);

    return 0;
}
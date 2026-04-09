#include <iostream>
#include <limits>
#include <stdexcept>

// Use unsigned long long for maximum standard integer size (up to 20!)
typedef unsigned long long ull;

/**
 * @brief Calculates the factorial of a non-negative integer iteratively.
 *
 * NOTE ON OVERFLOW: The factorial function grows extremely quickly. 
 * An unsigned long long can only accurately calculate up to 20!. 
 * For numbers larger than 20, a custom Big Integer class would be required.
 *
 * @param n The number to calculate the factorial for.
 * @return The factorial value (n!).
 * @throws std::invalid_argument if the input is negative.
 */
ull calculateFactorial(int n) {
    // 1. Input Validation
    if (n < 0) {
        // Throwing an exception is better practice than returning an arbitrary error value
        throw std::invalid_argument("Factorial is not defined for negative numbers.");
    }
    
    // 2. Handle Base Cases
    if (n == 0 || n == 1) {
        return 1;
    }

    // 3. Iterative Calculation
    ull result = 1;
    for (int i = 2; i <= n; ++i) {
        // We must cast 'i' to ull before multiplication to ensure the 
        // accumulator 'result' remains in the correct type domain.
        result *= static_cast<ull>(i);
    }
    
    return result;
}

/**
 * @brief Main function to handle user input and output.
 * @return 0 on successful execution.
 */
int main() {
    int number;
    
    std::cout << "=======================================================\n";
    std::cout << "             Factorial Calculator (n!)                \n";
    std::cout << "=======================================================\n";
    std::cout << "Enter a non-negative integer (Max Recommended: 20): ";
    
    // Read input
    if (!(std::cin >> number)) {
        std::cerr << "\n[ERROR] Invalid input type detected. Please enter an integer.\n";
        return 1;
    }
    
    try {
        // Call the calculation function
        ull result = calculateFactorial(number);
        
        // Output the result
        std::cout << "\n-------------------------------------------------------\n";
        std::cout << "The factorial of " << number << " (" << number << "! ) is: " << result << "\n";
        std::cout << "-------------------------------------------------------\n";

    } catch (const std::invalid_argument& e) {
        // Catch specific domain errors (negative numbers)
        std::cerr << "\n[RUNTIME ERROR] " << e.what() << "\n";
    } catch (...) {
        // Catch any other unexpected errors
        std::cerr << "\n[CRITICAL ERROR] An unknown error occurred during calculation.\n";
    }
    
    return 0;
}

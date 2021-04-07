#include "SGFilter.h"

namespace ORB_SLAM2
{

    // Default Constructor
    SGFilter::SGFilter(uint32_t poly_order, uint32_t filter_size) {

        // Always guarantee unique solution by imposing this condition
        if (poly_order + 1 > filter_size) {
	    while(1)
                std::cout << "SGFilter could not be initialized. Order should be smaller than number of filtered points" << std::endl;
        }

        // Assign attribute variables 
        poly_order_ = poly_order;
        filter_size_ = filter_size;

        current_line_ = 0;
        filter_iter_ = 0;

        A_.resize(filter_size_, poly_order_ + 1);

        // Initialize all vectors to zeros
        f_.resize(filter_size_);
        t_.resize(filter_size_);
        c_.resize(poly_order_ + 1);

        for (unsigned int i = 0; i < filter_size_; i++) {
            f_(i) = 0.0f;
            t_(i) = 0.0f;
        }

        //construct_A();
        
    }

    // Update the time and value vectors
    void SGFilter::update(float new_time_float, float new_val) {

        f_(current_line_) = new_val;
        t_(current_line_) = new_time_float;

        uint32_t line_np1 = current_line_;
        line_np1++;
        line_np1 = line_np1 % filter_size_;

        float tCenter = 0.5F*(new_time_float + t_(line_np1));

        // Push one slot forward
        for (uint32_t i = 0; i < filter_size_; i++)
        {
            for (uint32_t j = 0; j < poly_order_ + 1; j++) {           
                A_(i,j) = powFast(t_(i)-tCenter, j);
            }
        }

        if(filter_iter_ > filter_size_) {
            // Update the matrix and solve the least-squares prolem 
            // c_ = (A_.transpose() * A_).ldlt().solve(A_.transpose() * f_);   
            c_ = A_.fullPivHouseholderQr().solve(f_);

            // Reinitialize data 
            data_.yRaw = new_val;
            data_.y = 0.0f;
            data_.yDot = 0.0f;

            float tEnd = new_time_float-tCenter;
            // Update data filtered
            for (unsigned int i = 0; i <= poly_order_; i++) {
                data_.y += c_(i) * powFast(tEnd, i);
            }
            // Update data derivative
            for (unsigned int i = 1; i <= poly_order_; i++) {
                data_.yDot += i * c_(i) * powFast(tEnd, i - 1);
            }
        }

        filter_iter_++;
        current_line_++;
        current_line_ = current_line_ % filter_size_;
    }

    // Debugging Function: Print out Coefficient Matrix
    void SGFilter::debug_print_A(void) {

        unsigned int nrow = A_.rows();
        unsigned int ncol = A_.cols();

        std::cout << "nrow: " << nrow << std::endl;
        std::cout << "ncol: " << ncol << std::endl;

        for (unsigned int i = 0; i<nrow; i++)
        {
            for (unsigned int j=0; j<ncol; j++)
            {
                std::cout << A_(i,j) << ", ";   // print 6 decimal places
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    float SGFilter::powFast(float x, uint32_t n)
    {
        if(n==0)
        {
            return 1.0F;
        }
        else if(n==1)
        {
            return x;
        }
        else
        {
            float out = x;
            for(uint32_t i = 1; i<n; i++)
            {
                out*=x;
            }
            return out;
        }
    }
}

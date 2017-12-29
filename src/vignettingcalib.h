#ifndef VIGNETTINGCALIB_H
#define VIGNETTINGCALIB_H

#include "vignetting.h"
#include <opencv2/opencv.hpp>

namespace camera_model
{
class VignettingCalib : public vignetting
{
    class luminanceError
    {
        public:
        luminanceError( double _value, double _r )
        : value( _value )
        , r( _r )
        {
        }

        /* clang-format off */
      template< typename T >
      T luminance( T k0, T k2, T k4, T k6 ) const
      {
          return k0
              + k2 * r * r
              + k4 * r * r * r * r
              /*+ k6 * r * r * r * r * r * r*/ ;
      }
        /* clang-format on */

        template< typename T >
        bool operator( )( const T* const k, T* residuals ) const
        {
            T value_esti = luminance( k[0], k[1], k[2], k[3] );

            residuals[0] = T( value ) - value_esti;

            return true;
        }

        public:
        double value;
        double r;
    };

    public:
    VignettingCalib( ) {}
    VignettingCalib( cv::Size image_size, std::string camera_model_file, cv::Size boardSize, bool _is_color = false );

    void readin_points( const std::vector< std::pair< cv::Point2d, std::vector< double > > > points );
    void solve( );
    void showResualt( );
    void getValue9( std::vector< double >& value, const cv::Mat image, int x_index, int y_index );
    cv::Mat getPoints( cv::Mat image_in, std::vector< cv::Point2f > points );
    void addValue( cv::Mat& image, cv::Mat& image_color, double& index_x, double& index_y, int threshold );
    template< typename T >
    T avgInThree( T x1, T x2, T x3 )
    {
        return ( x1 + x2 + x3 ) / 3.0;
    }
    void inBoard( double& x_index, double& y_index );
    void drawRedPoint( cv::Mat& image_color, int x_index, int y_index );
    void drawYellowPoint( cv::Mat& image_color, int x_index, int y_index );
    void drawGreenPoint( cv::Mat& image_color, int x_index, int y_index );
};
}
#endif // VIGNETTINGCALIB_H

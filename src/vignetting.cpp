#include "vignetting.h"

void
camera_model::vignetting::addValue( cv::Mat& image, cv::Mat& image_color, double& index_x, double& index_y, int threshold )
{
    int value = image.at< uchar >( index_y, index_x );
    if ( value < threshold )
        drawRedPoint( image_color, index_x, index_y );
    else
    {
        value = getValue9( image, index_x, index_y );

        greyValues.push_back( value );
        double new_r = distance( index_x, index_y, center( 0 ), center( 1 ) );
        rs.push_back( new_r );
        drawGreenPoint( image_color, index_x, index_y );
    }
}

void
camera_model::vignetting::inBoard( double& x_index, double& y_index )
{
    x_index = x_index < 0.0 ? 0.0 : x_index;
    x_index = x_index > ( double )image_size.width ? ( double )image_size.width : x_index;

    y_index = y_index < 0.0 ? 0.0 : y_index;
    y_index = y_index > ( double )image_size.height ? ( double )image_size.height : y_index;
}

void
camera_model::vignetting::drawRedPoint( cv::Mat& image_color, int x_index, int y_index )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;
    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );
    cv::Scalar red( 0, 0, 255 );
    cv::circle( image_color,
                cv::Point( cvRound( x_index * drawMultiplier ), cvRound( y_index * drawMultiplier ) ),
                5,
                red,
                2,
                CV_AA,
                drawShiftBits );
}

void
camera_model::vignetting::drawYellowPoint( cv::Mat& image_color, int x_index, int y_index )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;
    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );
    cv::Scalar red( 0, 0, 255 );
    cv::circle( image_color,
                cv::Point( cvRound( x_index * drawMultiplier ), cvRound( y_index * drawMultiplier ) ),
                5,
                yellow,
                2,
                CV_AA,
                drawShiftBits );
}

void
camera_model::vignetting::drawGreenPoint( cv::Mat& image_color, int x_index, int y_index )
{
    int drawShiftBits  = 4;
    int drawMultiplier = 1 << drawShiftBits;
    cv::Scalar yellow( 0, 255, 255 );
    cv::Scalar green( 0, 255, 0 );
    cv::Scalar red( 0, 0, 255 );
    cv::circle( image_color,
                cv::Point( cvRound( x_index * drawMultiplier ), cvRound( y_index * drawMultiplier ) ),
                5,
                green,
                2,
                CV_AA,
                drawShiftBits );
}

int
camera_model::vignetting::getValue9( const cv::Mat image, int x_index, int y_index )
{
    int value
    = image.at< uchar >( y_index - 1, x_index - 1 ) + image.at< uchar >( y_index - 1, x_index )
      + image.at< uchar >( y_index - 1, x_index + 1 ) + image.at< uchar >( y_index, x_index - 1 )
      + image.at< uchar >( y_index, x_index ) + image.at< uchar >( y_index, x_index + 1 )
      + image.at< uchar >( y_index + 1, x_index - 1 ) + image.at< uchar >( y_index + 1, x_index )
      + image.at< uchar >( y_index + 1, x_index + 1 );
    return ( value / 9 );
}

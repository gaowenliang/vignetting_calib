#ifndef VIGNETTING_H
#define VIGNETTING_H

#include <camera_model/camera_models/CameraFactory.h>
#include <ceres/ceres.h>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace camera_model
{

class vignetting
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
                + k6 * r * r * r * r * r * r ;
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
    vignetting( ) {}
    vignetting( std::string camera_model_file, cv::Size boardSize )
    : chessbordSize( boardSize )
    {
        cam = camera_model::CameraFactory::instance( )->generateCameraFromYamlFile( camera_model_file );

        //        image_size = cv::Size( cam->imageWidth( ), cam->imageHeight( ) );
        image_size  = cv::Size( 1280, 1024 );
        center( 0 ) = image_size.width / 2;
        center( 1 ) = image_size.height / 2;
        //        center( 0 ) = 659.363; // image_size.width / 2;
        //        center( 1 ) = 516.62;  // image_size.height / 2;
    }

    public:
    void readin_points( const std::vector< std::pair< cv::Point2d, double > > points )
    {
        points_num = points.size( );
        greyValues.clear( );
        rs.clear( );

        for ( int point_index = 0; point_index < points_num; ++point_index )
        {
            double r = sqrt( ( points[point_index].first.x - center( 0 ) )
                             * ( points[point_index].first.x - center( 0 ) )
                             + ( points[point_index].first.y - center( 1 ) )
                               * ( points[point_index].first.y - center( 1 ) ) );

            rs.push_back( r );
            greyValues.push_back( points[point_index].second );
        }
    }

    double add( int raw_index, int col_index, double value ) {}
    double get( int xx, int yy )
    {
        double dis = distance( double( xx ), double( yy ), center( 0 ), center( 1 ) );
        //        std::cout << " dis " << dis << std::endl;
        double r = params[0] + params[1] * dis * dis + params[2] * dis * dis * dis * dis
                   + params[3] * dis * dis * dis * dis * dis * dis;

        return r;
    }

    cv::Mat remove( const cv::Mat image_in )
    {
        cv::Mat image_tmp( image_in.rows, image_in.cols, CV_8UC1 );
        for ( int raw_index = 0; raw_index < image_size.height; ++raw_index )
            for ( int col_index = 0; col_index < image_size.width; ++col_index )
            {
                double feed      = params[0] / get( col_index, raw_index );
                int velue        = image_in.at< uchar >( raw_index, col_index );
                int valuw_feeded = velue * feed;
                if ( valuw_feeded > 255 )
                    valuw_feeded = 255;
                if ( valuw_feeded < 0 )
                    valuw_feeded = 0;
                //                std::cout << " feed " << feed << std::endl;

                image_tmp.at< uchar >( raw_index, col_index ) = valuw_feeded;
            }
        return image_tmp;
    }

    void resualt( )
    {
        cv::Mat image( image_size, CV_8UC1, cv::Scalar( 0 ) );

        for ( int raw_index = 0; raw_index < image_size.height; ++raw_index )
            for ( int col_index = 0; col_index < image_size.width; ++col_index )
            {
                double value = get( col_index, raw_index );
                //                std::cout << " value " << value << std::endl;
                image.at< uchar >( raw_index, col_index ) = value;
            }

        cv::namedWindow( "resualt", cv::WINDOW_NORMAL );
        cv::imshow( "resualt", image );
        cv::waitKey( 0 );
    }

    void solve( )
    {

        if ( rs.size( ) == greyValues.size( ) )
            points_num = rs.size( );

        double poly_k[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

        ceres::Problem problem;

        for ( int i = 0; i < points_num; ++i )
        {
            ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction< luminanceError, 1, 4 >(
            new luminanceError( greyValues[i], rs[i] ) );

            problem.AddResidualBlock( costFunction, NULL /* squared loss */, poly_k );
        }

        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = true;
        options.trust_region_strategy_type   = ceres::DOGLEG;
        ceres::Solver::Summary summary;
        ceres::Solve( options, &problem, &summary );

        for ( int index   = 0; index < 7; ++index )
            params[index] = poly_k[index];

        std::cout << params[0] << " " << params[1] << " " << params[2] << " " << params[3] << " "
                  << params[4] /*<< " " << params[5] << " " << params[6] */
                  << std::endl;
    }

    template< typename T >
    T distance( T x1, T y1, T x2, T y2 )
    {
        return sqrt( ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 ) );
    }

    cv::Mat draw( )
    {
        cv::Mat image_show( image_size, CV_8UC3 );

        for ( int row_index = 0; row_index < image_show.rows; ++row_index )
            for ( int col_index = 0; col_index < image_show.cols; ++col_index )
            {
                double r = distance( double( col_index ), double( row_index ), center( 0 ), center( 1 ) );
            }
    }

    cv::Mat showPoly( ) { cv::Mat poly_image( ); }

    cv::Mat getPoints( cv::Mat image_in, std::vector< cv::Point2f > points )
    {
        //        cv::imshow( "raw", image_in );
        cv::Mat image = image_in;
        cv::Mat image_color;
        if ( image.channels( ) == 1 )
        {
            cv::cvtColor( image, image_color, CV_GRAY2RGB );
        }
        else
        {
            image.copyTo( image_color );
        }

        // for ( int point_index = 0; point_index < points.size( ); ++point_index )
        // {
        //     std::ostringstream oss;
        //     oss << point_index;
        //     cv::circle( image_color,
        //                 cv::Point( cvRound( points[point_index].x * drawMultiplier ),
        //                            cvRound( points[point_index].y * drawMultiplier ) ),
        //                 3, yellow, 2, CV_AA, drawShiftBits );
        //
        //     cv::putText( image_color, oss.str( ),
        //                  cv::Point( points[point_index].x, points[point_index].y ),
        //                  cv::FONT_HERSHEY_COMPLEX, 0.5, yellow, 1, CV_AA );
        // }

        //        std::cout << " points.size( ) " << points.size( ) << std::endl;

        int cnt = 0;
        for ( int raw_index = 0; raw_index < chessbordSize.height - 1; ++raw_index )
            for ( int col_index = 0; col_index < chessbordSize.width - 1; ++col_index )
            {
                int raw_index_add = chessbordSize.width * raw_index;
                ++cnt;

                double dx_raw = ( points[raw_index_add + col_index].x
                                  - points[raw_index_add + col_index + chessbordSize.width].x );
                double dy_raw = ( points[raw_index_add + col_index].y
                                  - points[raw_index_add + col_index + chessbordSize.width].y );
                double dx_col
                = ( points[raw_index_add + col_index].x - points[raw_index_add + col_index + 1].x );
                double dy_col
                = ( points[raw_index_add + col_index].y - points[raw_index_add + col_index + 1].y );

                double avg_x
                = ( points[raw_index_add + col_index].x + points[raw_index_add + col_index + 1].x
                    + points[raw_index_add + col_index + chessbordSize.width].x
                    + points[raw_index_add + col_index + chessbordSize.width + 1].x )
                  / 4;
                double avg_y
                = ( points[raw_index_add + col_index].y + points[raw_index_add + col_index + 1].y
                    + points[raw_index_add + col_index + chessbordSize.width].y
                    + points[raw_index_add + col_index + chessbordSize.width + 1].y )
                  / 4;

                double index_x;
                double index_y;

                int threshold = 60;

                if ( raw_index == 0 )
                {
                    index_x = avg_x + dx_raw;
                    index_y = avg_y + dy_raw;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }
                if ( col_index == 0 )
                {
                    index_x = avg_x + dx_col;
                    index_y = avg_y + dy_col;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }
                if ( raw_index == chessbordSize.height - 2 )
                {
                    index_x = avg_x - dx_raw;
                    index_y = avg_y - dy_raw;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }
                if ( col_index == chessbordSize.width - 2 )
                {
                    index_x = avg_x - dx_col;
                    index_y = avg_y - dy_col;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }
                if ( raw_index == 0 && col_index == 0 )
                {
                    index_x = avg_x + dx_raw + dx_col;
                    index_y = avg_y + dy_raw + dy_col;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }
                else if ( raw_index == 0 && col_index == chessbordSize.width - 2 )
                {
                    index_x = avg_x + dx_raw - dx_col;
                    index_y = avg_y + dy_raw - dy_col;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }
                else if ( raw_index == chessbordSize.height - 2 && col_index == 0 )
                {
                    index_x = avg_x - dx_raw + dx_col;
                    index_y = avg_y - dy_raw + dy_col;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }
                else if ( raw_index == chessbordSize.height - 2 && col_index == chessbordSize.width - 2 )
                {
                    index_x = avg_x - dx_raw - dx_col;
                    index_y = avg_y - dy_raw - dy_col;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }

                {
                    index_x = avg_x;
                    index_y = avg_y;
                    inBoard( index_x, index_y );
                    addValue( image, image_color, index_x, index_y, threshold );
                }
            }

        return image_color;
    }

    void addValue( cv::Mat& image, cv::Mat& image_color, double& index_x, double& index_y, int threshold );

    void inBoard( double& x_index, double& y_index );
    void drawRedPoint( cv::Mat& image_color, int x_index, int y_index );
    void drawYellowPoint( cv::Mat& image_color, int x_index, int y_index );
    void drawGreenPoint( cv::Mat& image_color, int x_index, int y_index );

    int getValue9( const cv::Mat image, int x_index, int y_index );

    public:
    camera_model::CameraPtr cam;
    double params[7];

    Eigen::Vector2d center;
    cv::Size image_size;
    cv::Size chessbordSize;

    int points_num;
    std::vector< double > greyValues;
    std::vector< double > rs;
};
}

#endif // VIGNETTING_H
